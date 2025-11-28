/**
 * @file ULP_EI_TTFM_Logger.ino
 * @brief Ultra-Low-Power Edge Impulse (EI) Data Logger for Cold-Start Monitoring.
 *
 * ----------------------------------------------------------------------------------
 * 🎯 PROJECT GOAL: Time-to-First-Measurement (TTFM) Analysis
 * ----------------------------------------------------------------------------------
 * Optimized for XIAO ESP32-S3 (based on ROM output: esp32s3-20210327).
 * This project measures the Time-to-First-Measurement (TTFM) from deep sleep wake-up
 * until the first valid Lux measurement is acquired.
 *
 * ----------------------------------------------------------------------------------
 * ⚙️ OPERATIONAL LOGIC
 * ----------------------------------------------------------------------------------
 * 1. WAKE: Node wakes up via the RTC Timer.
 * 2. INIT: Initializes Serial, I2C, and SD Card.
 * 3. CALIBRATE: Reads BMP180 calibration data only on a cold boot (power reset).
 * 4. TTFM START: Timer starts before BH1750 command is sent.
 * 5. MEASURE: Captures Lux (BH1750) and then Temp/Pressure (BMP180).
 * 6. TTFM STOP: Timer stops, calculating the TTFM_us.
 * 7. LOG: Logs data to SD Card.
 * 8. SLEEP: Enables RTC Timer wakeup and enters Deep Sleep.
 *
 * ----------------------------------------------------------------------------------
 * 📌 HARDWARE CONFIGURATION (XIAO ESP32-S3)
 * ----------------------------------------------------------------------------------
 * XIAO ESP32-S3 Pinout:
 * | Bus | Pin Function | GPIO Pin | Variable Name |
 * | :--- | :--- | :--- | :--- |
 * | I2C | SDA (Data) | 43 | SDA_PIN |
 * | I2C | SCL (Clock) | 44 | SCL_PIN |
 * | SPI | SD_CS (Chip Select) | 21 | SD_CS_PIN |
 * | SPI | SCK (Clock) | 7 | SD_SCK_PIN |
 * | SPI | MISO (Master In, Slave Out) | 8 | SD_MISO_PIN |
 * | SPI | MOSI (Master Out, Slave In) | 9 | SD_MOSI_PIN |
 *
 * NOTE: Pins 43, 44, 7, 8, 9, 21 are used based on the previous working setup.
 *
 * @author Gemini, Large Language Model
 * @date November 2025
 */

#include <Wire.h>
#include <SD.h>
#include <FS.h>
#include <SPI.h> 
#include "esp32-hal-log.h" 

// --- Configuration ---

// I2C PINS 
const int SDA_PIN = 43;
const int SCL_PIN = 44;

// SD CARD SPI PINS
const int SD_CS_PIN = 21;    // Chip Select (CS)
const int SD_SCK_PIN = 7;    // Clock (SCK)
const int SD_MISO_PIN = 8;   // Master In, Slave Out (MISO)
const int SD_MOSI_PIN = 9;   // Master Out, Slave In (MOSI)

// SLEEP CONFIGURATION
const uint64_t SLEEP_TIME_SEC = 60; // Wake up every 60 seconds (1 minute)
const char* LOG_FILE_NAME = "/ttfm_log.csv"; 

// BH1750 (Lux)
#define BH1750_I2C_ADDRESS 0x23 
#define BH1750_MEASURE_HR2_MODE 0x11 
const int BH1750_MEASUREMENT_TIME_MS = 180; // Max time for High Resolution Mode 2

// BMP180 (Temp/Pressure)
#define BMP_ADDR 0x77
#define OSS 3 // Oversampling Setting (0 to 3)

// Global variables
int current_lux = -1;
float current_temp_C = 0.0;
long current_pressure_Pa = 0;
unsigned long ttfm_us = 0; 

// BMP180 calibration variables (Stored in RTC memory for persistence across deep sleep)
RTC_DATA_ATTR long rtc_AC1, rtc_AC2, rtc_AC3;
RTC_DATA_ATTR unsigned long rtc_AC4, rtc_AC5, rtc_AC6;
RTC_DATA_ATTR long rtc_B1, rtc_B2, rtc_MB, rtc_MC, rtc_MD;
RTC_DATA_ATTR unsigned long rtc_logEntryCount = 0;

// --- CRITICAL FIX: Add a simple flag to skip deep sleep for first boot ---
// This is more robust than relying on time.
RTC_DATA_ATTR bool rtc_safeBootFlag = true; 


// --------------------------------------------------------
// --- Time-to-First-Measurement (TTFM) Functions ---
// --------------------------------------------------------

void printWakeupReason() {
    esp_sleep_source_t wakeup_reason = esp_sleep_get_wakeup_cause();
    Serial.print("Wakeup Cause: ");
    switch(wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("External signal using RTC_GPIO"); break;
        case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("External signal using RTC_GPIO"); break;
        case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Timer"); break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Touchpad"); break;
        case ESP_SLEEP_WAKEUP_ULP : Serial.println("ULP program"); break;
        case ESP_SLEEP_WAKEUP_UNDEFINED: Serial.println("Power-on Reset (Cold Boot)"); break;
        default : Serial.println("Other/Unspecified"); break;
    }
}

/**
 * @brief Enters ESP32 Deep Sleep mode.
 */
void goToDeepSleep() {
    Serial.printf("Entering Deep Sleep for %lu seconds...\n", (unsigned long)SLEEP_TIME_SEC);
    Serial.flush();
    
    // Enable wakeup source: RTC Timer
    esp_sleep_enable_timer_wakeup(SLEEP_TIME_SEC * 1000000ULL);
    
    // The ESP32-S3's native USB Serial connection requires careful timing.
    esp_deep_sleep_start();
}


// --------------------------------------------------------
// --- I2C Helper & BMP180 Functions (Unchanged) ---
// --------------------------------------------------------

int16_t readInt(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, 2);
  if (Wire.available() == 2) {
    return (Wire.read() << 8) | Wire.read();
  }
  return 0;
}

uint8_t readByte(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, 1);
  if (Wire.available() == 1) {
    return Wire.read();
  }
  return 0;
}

void readCalibration() {
    rtc_AC1 = readInt(BMP_ADDR, 0xAA);
    rtc_AC2 = readInt(BMP_ADDR, 0xAC);
    rtc_AC3 = readInt(BMP_ADDR, 0xAE);
    rtc_AC4 = readInt(BMP_ADDR, 0xB0);
    rtc_AC5 = readInt(BMP_ADDR, 0xB2);
    rtc_AC6 = readInt(BMP_ADDR, 0xB4);
    rtc_B1  = readInt(BMP_ADDR, 0xB6);
    rtc_B2  = readInt(BMP_ADDR, 0xB8);
    rtc_MB  = readInt(BMP_ADDR, 0xBA);
    rtc_MC  = readInt(BMP_ADDR, 0xBC);
    rtc_MD  = readInt(BMP_ADDR, 0xBE);
    Serial.println("BMP180 calibration data read and saved to RTC memory.");
}

long readRawTemperature() {
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0xF4); 
    Wire.write(0x2E); 
    Wire.endTransmission();
    delay(5); 
    return readInt(BMP_ADDR, 0xF6); 
}

float calculateTemperature(long UT, long &B5) {
    long X1 = ((UT - rtc_AC6) * rtc_AC5) >> 15;
    long X2 = (rtc_MC << 11) / (X1 + rtc_MD);
    B5 = X1 + X2;
    return (float)((B5 + 8) >> 4) / 10.0;
}

long readRawPressure() {
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0xF4); 
    Wire.write(0x34 + (OSS << 6)); 
    Wire.endTransmission();
    delay(26); 
    
    long up = readInt(BMP_ADDR, 0xF6) & 0xFFFF; 
    up <<= 8;
    up |= readByte(BMP_ADDR, 0xF8); 
    up >>= (8 - OSS); 
    return up;
}

long calculatePressure(long UP, long B5) {
    long B6 = B5 - 4000;
    long X1 = (rtc_B2 * ((B6 * B6) >> 12)) >> 11;
    long X2 = (rtc_AC2 * B6) >> 11;
    long X3 = X1 + X2;
    long B3 = ((((long)rtc_AC1 * 4 + X3) << OSS) + 2) >> 2;

    X1 = (rtc_AC3 * B6) >> 13;
    X2 = (rtc_B1 * ((B6 * B6) >> 12)) >> 16;
    long X3_temp = ((X1 + X2) + 2) >> 2; 
    unsigned long B4 = (rtc_AC4 * (unsigned long)(X3_temp + 32768)) >> 15;
    
    unsigned long B7 = ((unsigned long)UP - B3) * (50000 >> OSS);
    
    long p;
    if (B7 < 0x80000000) p = (B7 * 2) / B4;
    else p = (B7 / B4) * 2;

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);
    
    return p;
}

// --------------------------------------------------------
// --- Data Acquisition (Lux) Functions (Unchanged) ---
// --------------------------------------------------------

void bh1750StartMeasurement() {
    Wire.beginTransmission(BH1750_I2C_ADDRESS);
    Wire.write(0x01); // Power On
    Wire.endTransmission();
    
    Wire.beginTransmission(BH1750_I2C_ADDRESS);
    Wire.write(BH1750_MEASURE_HR2_MODE); // High Resolution Mode 2 (Max 180ms)
    Wire.endTransmission();
}

int bh1750ReadLux() {
    if (Wire.requestFrom(BH1750_I2C_ADDRESS, 2) == 2) {
        int msb = Wire.read();
        int lsb = Wire.read();
        
        unsigned int raw_value = (msb << 8) | lsb;
        
        return (int)(raw_value / 1.2);
    } else {
        log_e("BH1750 Read Error: NACK detected.");
        return -1; 
    }
}

bool captureSensorDataWithTTFM() {
    
    Wire.begin(SDA_PIN, SCL_PIN); 
    
    unsigned long start_time_us = esp_timer_get_time();
    
    bh1750StartMeasurement();
    
    delay(BH1750_MEASUREMENT_TIME_MS);
    
    current_lux = bh1750ReadLux();
    
    ttfm_us = esp_timer_get_time() - start_time_us;
    
    if (current_lux < 0) {
        log_e("TTFM measurement failed due to Lux sensor read error.");
        return false;
    }
    
    long UT = readRawTemperature(); 
    long B5; 
    current_temp_C = calculateTemperature(UT, B5); 
    
    long UP = readRawPressure(); 
    current_pressure_Pa = calculatePressure(UP, B5); 
    
    return true;
}


// --------------------------------------------------------
// --- SD Card Logging Functions (Unchanged) ---
// --------------------------------------------------------

void appendFile(fs::FS &fs, const char * path, const char * message) {
    File file = fs.open(path, FILE_APPEND);
    if(!file){
        log_e("Failed to open file for appending: %s", path);
        return;
    }
    if(file.print(message)){
        // Success
    } else {
        log_e("Append failed for file: %s", path);
    }
    file.close();
}

void logSensorData() {
    char dataString[160];
    unsigned long timestamp_ms = millis(); 
    
    // ML-Ready CSV Row: Timestamp, Lux, Temp, Pressure, TTFM
    sprintf(dataString, "%lu,%d,%.2f,%ld,%lu\n", 
            timestamp_ms, 
            current_lux, 
            current_temp_C,
            current_pressure_Pa,
            ttfm_us);
    
    Serial.printf("LOG ENTRY %lu (TTFM: %lu us): %s", rtc_logEntryCount, ttfm_us, dataString);

    appendFile(SD, LOG_FILE_NAME, dataString);

    rtc_logEntryCount++;
}


// --------------------------------------------------------
// --- Arduino Setup and Loop ---
// --------------------------------------------------------

void setup() {
    // 1. Initialize Serial & Wait
    Serial.begin(115200);
    // CRITICAL: Wait a few seconds for the computer to recognize the USB CDC port
    // This is vital for the ESP32-S3 Native USB.
    unsigned long startTime = millis();
    // Loop until Serial is ready OR 3 seconds have passed.
    while (!Serial && (millis() - startTime < 3000)) { 
        delay(10); 
    }
    
    printWakeupReason();

    // --- Safe Boot Logic ---
    if (rtc_safeBootFlag) {
        Serial.println("--- SAFE BOOT: Deep Sleep is DISABLED for this cycle. ---");
        Serial.println("Please enter 'C' in the Serial Monitor to CONTINUE (enable deep sleep).");
    }

    // 2. Initialize SD Card
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD Card Mount Failed. Continuing without logging.");
    } else {
        Serial.println("SD Card mounted successfully.");
    }

    // 🌟 TEST CONFIRMATION PRINT 🌟
    Serial.println("====================================================");
    Serial.println("✅ XIAO ESP32-S3 Logger Initialized and Ready to Measure.");
    Serial.println("====================================================");

    // 3. Read Calibration Data on Cold Boot
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
        Wire.begin(SDA_PIN, SCL_PIN); 
        readCalibration();
        // After cold boot calibration, proceed to normal logging
        rtc_safeBootFlag = false; 
    } else {
        Serial.printf("Woke from sleep. Current log count: %lu\n", rtc_logEntryCount);
    }

    // 4. Write CSV Header (only if file doesn't exist)
    if (rtc_logEntryCount == 0) {
        appendFile(SD, LOG_FILE_NAME, "Timestamp_ms,Lux_Value,Temperature_C,Pressure_Pa,TTFM_us\n");
        Serial.println("Created new log file with header.");
    }
    
    // 5. CAPTURE DATA and TTFM
    Serial.println("Starting measurement and TTFM capture...");
    if (captureSensorDataWithTTFM()) {
        logSensorData();
    } else {
        Serial.println("Measurement failed. Skipping log entry.");
    }
    
    // 6. Conditional Deep Sleep
    if (!rtc_safeBootFlag) {
        // If the flag is FALSE (meaning we are running normally or the user cleared the flag)
        goToDeepSleep();
    }
    // If rtc_safeBootFlag is TRUE, the program will fall through to loop() and wait.
}

// The loop waits indefinitely until the user sends a command to proceed or re-flash.
void loop() {
    if (rtc_safeBootFlag) {
        // This stall is only active after a flash and before the user sends 'C'
        if (Serial.available()) {
            char command = Serial.read();
            if (command == 'C' || command == 'c') {
                Serial.println("Manual 'C' received. Enabling Deep Sleep and proceeding.");
                rtc_safeBootFlag = false; // Clear the flag
                goToDeepSleep(); // Force deep sleep
            }
        }
        delay(100); 
    } else {
        // This part should be unreachable if goToDeepSleep() was called in setup
        // But included as a fail-safe to prevent unnecessary power draw.
        goToDeepSleep(); 
    }
}

