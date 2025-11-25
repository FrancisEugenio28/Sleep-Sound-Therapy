#include <Arduino.h>
#include "BluetoothA2DPSink.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include "RTClib.h"
#include "driver/i2s.h"

// --- PINS ---
#define RADAR_PIN       35
#define BATTERY_PIN     34
#define AMP_SD_PIN      16  // Amp Shutdown

// I2S Amp Pins (Port 1)
#define I2S_BCK         26
#define I2S_WS          25
#define I2S_DATA        27

// I2S Mic Pins (Port 0)
#define MIC_WS          33
#define MIC_SCK         32
#define MIC_SD          19

// --- BLE UUIDS (Must match Flutter App) ---
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // Write (Phone -> ESP)
#define CHARACTERISTIC_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // Notify (ESP -> Phone)

// --- GLOBALS ---
BluetoothA2DPSink a2dp_sink;
BLECharacteristic *pTxCharacteristic;
RTC_DS3231 rtc;
bool deviceConnected = false;

// Logic Vars
const unsigned long MIN_MOTION_DURATION = 3000;
const unsigned long SLEEP_TIMEOUT_MS = 30 * 60 * 1000;
volatile bool userIsAsleep = false;
unsigned long lastMotionTime = 0;
unsigned long motionStartTime = 0;
bool isRawSignalHigh = false;
bool confirmedMotion = false;

// --- BLE SERVER CALLBACKS ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; pServer->startAdvertising(); }
};

// --- BLE WRITE CALLBACK (Receive Commands) ---
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = String(pCharacteristic->getValue().c_str());
      rxValue.trim();
      
      if (rxValue.length() > 0) {
        if (rxValue.startsWith("VOL:")) {
           int vol = rxValue.substring(4).toInt();
           // Map 0-100 to 0-127 (A2DP standard)
           int a2dpVol = map(vol, 0, 100, 0, 127);
           a2dp_sink.set_volume(a2dpVol);
        }
        else if (rxValue.equalsIgnoreCase("RST")) {
           userIsAsleep = false;
           lastMotionTime = millis();
           digitalWrite(AMP_SD_PIN, HIGH); // Wake up amp
        }
        // Diagnostics are handled in the main loop reporting
      }
    }
};

void setupI2S_Mic() {
  i2s_config_t i2s_mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t mic_pins = {
    .bck_io_num = MIC_SCK,
    .ws_io_num = MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = MIC_SD
  };
  i2s_driver_install(I2S_NUM_0, &i2s_mic_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &mic_pins);
}

void setup() {
  Serial.begin(115200);

  // 1. Init Pins
  pinMode(RADAR_PIN, INPUT);
  pinMode(BATTERY_PIN, INPUT);
  analogSetAttenuation(ADC_11db);
  
  pinMode(AMP_SD_PIN, OUTPUT);
  digitalWrite(AMP_SD_PIN, HIGH); 

  // 2. Init RTC
  if (!rtc.begin()) Serial.println("RTC Fail");
  if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // 3. Init BLE (Data)
  BLEDevice::init("SmartSleep_Data"); // Name for App
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE Started.");

  // 4. Init Mic
  setupI2S_Mic();

  // 5. Init A2DP (Speaker)
  // We force A2DP to use I2S_NUM_1 to avoid conflict with Mic on NUM_0
  i2s_pin_config_t amp_pins = {
      .bck_io_num = I2S_BCK,
      .ws_io_num = I2S_WS,
      .data_out_num = I2S_DATA,
      .data_in_num = I2S_PIN_NO_CHANGE
  };
  a2dp_sink.set_pin_config(amp_pins);
  a2dp_sink.start("SmartSleep_Speaker"); // Name for Bluetooth Settings
  a2dp_sink.set_volume(100); 
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. RADAR LOGIC
  if (digitalRead(RADAR_PIN) == HIGH) {
    if (!isRawSignalHigh) {
      isRawSignalHigh = true;
      motionStartTime = currentMillis;
    } else if ((currentMillis - motionStartTime) > MIN_MOTION_DURATION) {
      confirmedMotion = true;
      lastMotionTime = currentMillis;
      if (userIsAsleep) {
        userIsAsleep = false;
        digitalWrite(AMP_SD_PIN, HIGH); // Wake Amp
      }
    }
  } else {
    isRawSignalHigh = false;
    confirmedMotion = false;
  }

  // 2. SLEEP TIMER
  if (!userIsAsleep && (currentMillis - lastMotionTime > SLEEP_TIMEOUT_MS)) {
    userIsAsleep = true;
    digitalWrite(AMP_SD_PIN, LOW); // Kill Amp Power
  }

  // 3. DATA REPORTING VIA BLE (Every 1 sec)
  static unsigned long lastReport = 0;
  if (deviceConnected && (currentMillis - lastReport > 1000)) {
    lastReport = currentMillis;
    DateTime now = rtc.now();
    
    // Battery
    float volts = analogRead(BATTERY_PIN) * (3.3 / 4095.0) * 2.0;

    // Mic Level (Quick Sample)
    int32_t micSample[64];
    size_t bytes_read = 0;
    i2s_read(I2S_NUM_0, &micSample, sizeof(micSample), &bytes_read, 0);
    int micLevel = (bytes_read > 0) ? abs(micSample[0]) : 0;

    // Format: "Time|Motion|SleepIn|Volts|Mic"
    char msg[64];
    long timeLeft = (SLEEP_TIMEOUT_MS - (currentMillis - lastMotionTime))/1000;
    if(timeLeft<0) timeLeft=0;
    
    snprintf(msg, 64, "DATA:%02d:%02d:%02d|%s|%ld|%.2fV|%d", 
      now.hour(), now.minute(), now.second(),
      (confirmedMotion ? "YES" : "NO"),
      timeLeft, volts, micLevel
    );

    pTxCharacteristic->setValue(msg);
    pTxCharacteristic->notify();
  }
  
  delay(20); // Yield to A2DP task
}