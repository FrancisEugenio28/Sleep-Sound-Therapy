/*Sound Therapy Device Firmware*/

#include <Arduino.h>
#include <Wire.h>
#include "RTClib.h"
#include "driver/i2s.h"
#include "math.h"
#include "BluetoothSerial.h"

// =======================
// --- PIN DEFINITIONS ---
// =======================

#define RADAR_PIN       35  
#define BATTERY_PIN     34  

// I2S Amp (MAX98357A) - Port 1
#define I2S_AMP_PORT    I2S_NUM_1
#define I2S_AMP_LRCK    25  
#define I2S_AMP_BCLK    26  
#define I2S_AMP_DIN     27  
#define I2S_AMP_SD      16  

// I2S Mic (SPH0645) - Port 0
#define I2S_MIC_PORT    I2S_NUM_0
#define I2S_MIC_WS      33  
#define I2S_MIC_SCK     32  
#define I2S_MIC_SD      19  

// ==============================
// --- CONFIGURATION SETTINGS ---
// ==============================

#define SAMPLE_RATE     44100
#define FREQUENCY       440.0   
#define BUFFER_LEN      128

// Sleep Logic
const unsigned long MIN_MOTION_DURATION = 3000;    // 3.0 Seconds
const unsigned long SLEEP_TIMEOUT_MS = 30 * 60 * 1000; // 30 Minutes

// Sensor Fusion Settings (New)
const int MIC_THRESHOLD = 5000; // Sensitivity above baseline
volatile int micBaseline = 0;   // Stores room noise level

const int REPORT_INTERVAL = 180000; // 3minutes

// ========================
// --- GLOBAL VARIABLES ---
// ========================

volatile bool systemEnabled = true;     
volatile bool userIsAsleep = false;     
volatile int currentVolume = 50;        
volatile unsigned long lastMotionTime = 0; 

BluetoothSerial SerialBT;
RTC_DS3231 rtc;

TaskHandle_t AudioTaskHandle;
TaskHandle_t LogicTaskHandle;

// Prototypes
void Task_Audio(void *pvParameters);
void Task_Logic(void *pvParameters);
void setupI2S_Amp();
void setupI2S_Mic();
float readBatteryVoltage();
void performCalibration(); // New function

// =============
// --- SETUP ---
// =============
void setup() {
  Serial.begin(115200);

  // 1. Init Bluetooth
  // NAME CHANGED to force phone to refresh cache
  // PIN REMOVED to fix connection hanging
  SerialBT.begin("SmartSleep_Pro"); 
  Serial.println("Bluetooth Started. Ready to Pair.");

  // 2. Init RTC
  if (!rtc.begin()) {
    Serial.println("Error: RTC Not Found");
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // 3. Init Pins
  pinMode(RADAR_PIN, INPUT);
  
  // Configure ADC for Battery (11db attenuation for full 3.3V range)
  analogSetAttenuation(ADC_11db);
  pinMode(BATTERY_PIN, INPUT);

  // Amp Shutdown Pin (High = Enable)
  pinMode(I2S_AMP_SD, OUTPUT);
  digitalWrite(I2S_AMP_SD, HIGH);

  // 4. CALIBRATION
  setupI2S_Mic(); 
  performCalibration(); 

  // 5. Launch FreeRTOS Tasks
  xTaskCreatePinnedToCore(Task_Logic, "LogicTask", 8192, NULL, 1, &LogicTaskHandle, 1);
  xTaskCreatePinnedToCore(Task_Audio, "AudioTask", 10240, NULL, 2, &AudioTaskHandle, 0);
}

void loop() { vTaskDelay(1000); }

// ============================
// --- TASK 1: AUDIO ENGINE ---
// ============================
void Task_Audio(void *pvParameters) {
  setupI2S_Amp();
  
  static float phase = 0.0;
  float phase_step = (2.0 * PI * FREQUENCY) / SAMPLE_RATE;
  int16_t samples[BUFFER_LEN * 2];
  size_t bytes_written = 0;

  for(;;) {
    if (systemEnabled && !userIsAsleep) {
      digitalWrite(I2S_AMP_SD, HIGH);
      for (int i = 0; i < BUFFER_LEN; i++) {
        float raw_val = sin(phase);
        int16_t amplitude = (currentVolume / 100.0) * 20000; 
        int16_t final_sample = (int16_t)(raw_val * amplitude);
        samples[i*2 + 0] = final_sample; 
        samples[i*2 + 1] = final_sample; 
        phase += phase_step;
        if (phase >= 2.0 * PI) phase -= 2.0 * PI;
      }
      i2s_write(I2S_AMP_PORT, samples, sizeof(samples), &bytes_written, portMAX_DELAY);
    } else {
      digitalWrite(I2S_AMP_SD, LOW); 
      i2s_zero_dma_buffer(I2S_AMP_PORT); 
      vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
  }
}

// ===============================
// --- TASK 2: LOGIC & SENSORS ---
// ===============================
void Task_Logic(void *pvParameters) {
  // Note: Mic already setup in setup() for calibration
  
  unsigned long lastReportTime = 0;
  unsigned long motionStartTime = 0;
  bool isRawSignalHigh = false;
  bool confirmedMotion = false;

  for(;;) {
    unsigned long currentMillis = millis();

    // --- A. BLUETOOTH ---
    if (SerialBT.available()) {
      String command = SerialBT.readStringUntil('\n');
      command.trim(); 
      
      if (command.startsWith("VOL:")) {
        int newVol = command.substring(4).toInt();
        currentVolume = constrain(newVol, 0, 100);
        SerialBT.print("MSG:Volume Set to "); SerialBT.println(currentVolume);
      }
      else if (command.equalsIgnoreCase("RST")) {
        lastMotionTime = currentMillis;
        userIsAsleep = false;
        SerialBT.println("MSG:Timer Reset. Audio ON.");
      }
      else if (command.equalsIgnoreCase("DIAG")) {
        float volts = readBatteryVoltage();
        SerialBT.print("DIAG:Battery:"); SerialBT.print(volts); SerialBT.println("V");
      }
    }

    // --- B. RADAR LOGIC ---
    int pinState = digitalRead(RADAR_PIN);
    if (pinState == HIGH) {
      if (!isRawSignalHigh) {
        isRawSignalHigh = true;
        motionStartTime = currentMillis;
      } else {
        if ((currentMillis - motionStartTime) > MIN_MOTION_DURATION) {
          confirmedMotion = true;
          lastMotionTime = currentMillis; 
          if (userIsAsleep) {
            userIsAsleep = false;
            SerialBT.println("MSG:Motion Detected! Audio Resuming.");
          }
        }
      }
    } else {
      isRawSignalHigh = false;
      confirmedMotion = false;
    }

    // --- C. READ MIC (Moved UP so we can use it in logic) ---
    int32_t micSample[64];
    size_t bytes_read = 0;
    i2s_read(I2S_MIC_PORT, &micSample, sizeof(micSample), &bytes_read, 0);
    int32_t micLevel = 0;
    if(bytes_read > 0) { micLevel = abs(micSample[0]); }

    // --- D. SENSOR FUSION SLEEP LOGIC ---
    bool timeIsUp = (currentMillis - lastMotionTime > SLEEP_TIMEOUT_MS);
    bool roomIsQuiet = (micLevel < (micBaseline + MIC_THRESHOLD));

    if (!userIsAsleep && timeIsUp) {
      if (roomIsQuiet) {
        // Both Sensors Agree: User is likely asleep
        userIsAsleep = true;
        SerialBT.println("MSG:Sleep Confirmed (Time+Mic). Audio OFF.");
      } else {
        // Time is up, but Mic hears noise
        // We extend the timer slightly (don't turn off yet)
        // We only print this occasionally to avoid spamming
        if (currentMillis % 5000 < 50) { 
           SerialBT.println("MSG:Time up, but Room Noisy. Waiting...");
        }
        // Push the "last motion" forward so we check again in 10 seconds
        lastMotionTime = currentMillis - SLEEP_TIMEOUT_MS + 10000; 
      }
    }

    // --- E. REPORTING ---
    if (currentMillis - lastReportTime >= REPORT_INTERVAL) {
      lastReportTime = currentMillis;
      DateTime now = rtc.now();
      
      long timeLeft = (SLEEP_TIMEOUT_MS - (currentMillis - lastMotionTime)) / 1000;
      if (timeLeft < 0) timeLeft = 0;
      if (userIsAsleep) timeLeft = 0;

      SerialBT.printf("DATA:%02d:%02d:%02d|%s|%ld|%d|%d\n",
        now.hour(), now.minute(), now.second(),
        (confirmedMotion ? "YES" : "NO"),
        timeLeft,
        micLevel,
        currentVolume
      );
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); 
  }
}

// ========================
// --- HELPER FUNCTIONS ---
// ========================

void performCalibration() {
  Serial.println("Calibrating Mic...");
  // Use SerialBT only if connected, otherwise just Serial
  long total = 0;
  int samples = 0;
  unsigned long start = millis();
  
  while (millis() - start < 5000) {
    int32_t micSample[64];
    size_t bytes_read = 0;
    i2s_read(I2S_MIC_PORT, &micSample, sizeof(micSample), &bytes_read, 0);
    if (bytes_read > 0) {
       total += abs(micSample[0]);
       samples++;
    }
    delay(10);
  }
  
  if (samples > 0) micBaseline = total / samples;
  else micBaseline = 10000; 
  
  Serial.print("Calibration Done. Baseline: ");
  Serial.println(micBaseline);
}

float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  return raw * (3.3 / 4095.0) * 2.0;
}

void setupI2S_Amp() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_AMP_BCLK,
    .ws_io_num = I2S_AMP_LRCK,
    .data_out_num = I2S_AMP_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_AMP_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_AMP_PORT, &pin_config);
  i2s_set_sample_rates(I2S_AMP_PORT, SAMPLE_RATE);
}

void setupI2S_Mic() {
  i2s_config_t i2s_config = {
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
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_MIC_SCK,
    .ws_io_num = I2S_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SD
  };
  i2s_driver_install(I2S_MIC_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_MIC_PORT, &pin_config);
}