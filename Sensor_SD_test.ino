/**
 * @file SD_Card_Data_Logger.ino
 * @brief ESP32 Data Logger for BH1750 (Lux) and BMP180 (Temp/Pressure) to SD Card.
 *
 * This version uses custom pin configurations for both I2C and SPI, 
 * incorporating the complex calculation logic required for the BMP180 sensor.
 * * --- Pin Configuration Used (Confirmed Working) ---
 * I2C (SDA/SCL): GPIO 43 / GPIO 44
 * SPI (CS/SCK/MISO/MOSI): 21 / 7 / 8 / 9
 * * LOG FORMAT: Timestamp_ms,Lux_Value,Temperature_C,Pressure_Pa
 */

#include <Wire.h>
#include <SD.h>
#include <FS.h>
#include <SPI.h> 

// --- Pin Configuration ---
const int SDA_PIN = 43;
const int SCL_PIN = 44;

const int SD_CS_PIN = 21;    // Chip Select (CS)
const int SD_SCK_PIN = 7;    // Clock (SCK)
const int SD_MISO_PIN = 8;   // Master In, Slave Out (MISO)
const int SD_MOSI_PIN = 9;   // Master Out, Slave In (MOSI)

// --- Sensor Configuration ---

// BH1750 (Lux)
#define BH1750_I2C_ADDRESS 0x23 
#define BH1750_MEASURE_HR2_MODE 0x11 

// BMP180 (Temp/Pressure)
#define BMP_ADDR 0x77
#define OSS 3 // Oversampling Setting (0 to 3)

// File to log data to
const char* LOG_FILE_NAME = "/log.csv"; 

// Global variables for sensor data
int current_lux = 0;
float current_temp_C = 0.0;
long current_pressure_Pa = 0;
unsigned long logEntryCount = 0;

// BMP180 calibration variables (Read from the sensor's EEPROM on startup)
int16_t bmp_AC1, bmp_AC2, bmp_AC3;
uint16_t bmp_AC4, bmp_AC5, bmp_AC6;
int16_t bmp_B1, bmp_B2, bmp_MB, bmp_MC, bmp_MD;

// --------------------------------------------------------
// --- I2C Helper Function ---
// --------------------------------------------------------

/**
 * @brief Reads a 16-bit value from a register address on an I2C device.
 */
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

/**
 * @brief Reads an 8-bit value from a register address on an I2C device.
 */
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

// --------------------------------------------------------
// --- BH1750 (Lux) Functions ---
// --------------------------------------------------------

void bh1750Init() {
    // Initialize I2C with the custom pins (43 and 44)
    Wire.begin(SDA_PIN, SCL_PIN); 
    
    // Send Power On command (0x01)
    Wire.beginTransmission(BH1750_I2C_ADDRESS);
    Wire.write(0x01);
    Wire.endTransmission();
    
    // Set to High Resolution Mode 2 (0x11)
    Wire.beginTransmission(BH1750_I2C_ADDRESS);
    Wire.write(BH1750_MEASURE_HR2_MODE); 
    Wire.endTransmission();
    
    Serial.println("BH1750 sensor initialized on I2C.");
}

int bh1750ReadLux() {
    // Wait for the measurement to complete (max 180ms for HR2 mode)
    delay(180); 

    if (Wire.requestFrom(BH1750_I2C_ADDRESS, 2) == 2) {
        int msb = Wire.read();
        int lsb = Wire.read();
        
        unsigned int raw_value = (msb << 8) | lsb;
        
        // Convert raw value to Lux (HR2 mode has a divisor of 1.2)
        int lux = (int)(raw_value / 1.2);
        
        return lux;
    } else {
        Serial.println("Error reading from BH1750.");
        return -1; // Return error value
    }
}

// --------------------------------------------------------
// --- BMP180 (Temp/Pressure) Functions ---
// --------------------------------------------------------

/**
 * @brief Reads the calibration coefficients from the BMP180 EEPROM.
 */
void readCalibration() {
    bmp_AC1 = readInt(BMP_ADDR, 0xAA);
    bmp_AC2 = readInt(BMP_ADDR, 0xAC);
    bmp_AC3 = readInt(BMP_ADDR, 0xAE);
    bmp_AC4 = readInt(BMP_ADDR, 0xB0);
    bmp_AC5 = readInt(BMP_ADDR, 0xB2);
    bmp_AC6 = readInt(BMP_ADDR, 0xB4);
    bmp_B1  = readInt(BMP_ADDR, 0xB6);
    bmp_B2  = readInt(BMP_ADDR, 0xB8);
    bmp_MB  = readInt(BMP_ADDR, 0xBA);
    bmp_MC  = readInt(BMP_ADDR, 0xBC);
    bmp_MD  = readInt(BMP_ADDR, 0xBE);
    Serial.println("BMP180 calibration data read.");
}

/**
 * @brief Reads the raw Uncompensated Temperature (UT) value.
 * @return The raw temperature reading.
 */
long readRawTemperature() {
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0xF4); // Control register
    Wire.write(0x2E); // Command to measure temperature (0x2E)
    Wire.endTransmission();
    delay(5); // Wait 4.5ms
    return readInt(BMP_ADDR, 0xF6); // Read result
}

/**
 * @brief Reads the raw Uncompensated Pressure (UP) value.
 * @return The raw pressure reading.
 */
long readRawPressure() {
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0xF4); // Control register
    // Command to measure pressure, using OSS 3 (0x34 + 3*2) = 0x74
    Wire.write(0x34 + (OSS << 6)); 
    Wire.endTransmission();
    
    // Wait for measurement to complete (max 25.5ms for OSS=3)
    delay(26); 
    
    // Read MSB, LSB, XLSB from 0xF6, 0xF7, 0xF8
    long up = readInt(BMP_ADDR, 0xF6) & 0xFFFF;
    up <<= 8;
    up |= readByte(BMP_ADDR, 0xF8);
    
    // Shift result according to OSS
    up >>= (8 - OSS); 
    return up;
}

/**
 * @brief Calculates compensated temperature in degrees C.
 * @param UT Raw uncompensated temperature.
 * @param B5 Compensation variable from temp calculation (passed by reference).
 * @return Compensated temperature in degrees C.
 */
float calculateTemperature(long UT, long &B5) {
    long X1 = ((UT - bmp_AC6) * bmp_AC5) >> 15;
    long X2 = (bmp_MC << 11) / (X1 + bmp_MD);
    B5 = X1 + X2;
    // T = (B5 + 8) / 16, then convert to float for C
    return (float)((B5 + 8) >> 4) / 10.0;
}

/**
 * @brief Calculates compensated pressure in Pascals (Pa).
 * @param UP Raw uncompensated pressure.
 * @param B5 Compensation variable (calculated during temp calculation).
 * @return Compensated pressure in Pascals (Pa).
 */
long calculatePressure(long UP, long B5) {
    long B6 = B5 - 4000;
    
    // Calculate B3
    long X1 = (bmp_B2 * ((B6 * B6) >> 12)) >> 11;
    long X2 = (bmp_AC2 * B6) >> 11;
    long X3 = X1 + X2;
    long B3 = ((((long)bmp_AC1 * 4 + X3) << OSS) + 2) >> 2;

    // Calculate B4
    X1 = (bmp_AC3 * B6) >> 13;
    X2 = (bmp_B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    unsigned long B4 = (bmp_AC4 * (unsigned long)(X3 + 32768)) >> 15;
    
    // Calculate P
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
// --- SD Card Logging Functions ---
// --------------------------------------------------------

/**
 * @brief Writes data (string) to the specified file on the SD card.
 */
void appendFile(fs::FS &fs, const char * path, const char * message) {
    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        // Log success
    } else {
        Serial.println("- Append failed");
    }
    file.close();
}

/**
 * @brief Saves the current sensor readings to the SD card.
 */
void logSensorData() {
    // 1. Read Lux
    current_lux = bh1750ReadLux();

    // 2. Read Temp and Pressure (requires multi-step process)
    long UT = readRawTemperature(); // Read raw temperature first
    long B5; // Compensation variable
    current_temp_C = calculateTemperature(UT, B5); // Calculate temperature (and B5)
    
    long UP = readRawPressure(); // Read raw pressure
    current_pressure_Pa = calculatePressure(UP, B5); // Calculate pressure

    // Only log if the lux reading was successful
    if (current_lux < 0) {
        Serial.println("Skipping log entry due to I2C read error on BH1750.");
        return;
    }
    
    // 3. Format the data string: TIMESTAMP,LUX,TEMP,PRESSURE\n
    char dataString[128];
    unsigned long timestamp = millis();
    
    // Note: We use 2 decimal places for temperature (%.2f) and no decimal for lux/pressure
    sprintf(dataString, "%lu,%d,%.2f,%ld\n", 
            timestamp, 
            current_lux, 
            current_temp_C,
            current_pressure_Pa);
    
    // Print to Serial for immediate verification
    Serial.printf("LOG ENTRY %lu: %s", logEntryCount, dataString);

    // 4. Write to SD card
    appendFile(SD, LOG_FILE_NAME, dataString);

    logEntryCount++;
}

// --------------------------------------------------------
// --- Arduino Setup and Loop ---
// --------------------------------------------------------

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    // 1. Initialize BH1750 (and I2C bus with custom pins 43/44)
    bh1750Init();
    
    // 2. Read BMP180 Calibration Data
    readCalibration();

    // 3. Initialize SD Card with custom SPI pins
    // Configure the SPI bus with the specific pins: SCK (7), MISO (8), MOSI (9), CS (21)
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("SD Card Mount Failed. Check wiring and CS pin.");
        while (true) { delay(1000); }
    }
    Serial.println("SD Card mounted successfully.");

    // 4. Write CSV Header 
    File file = SD.open(LOG_FILE_NAME, FILE_READ);
    if (!file) {
        // File does not exist, write the header with all three new values
        appendFile(SD, LOG_FILE_NAME, "Timestamp_ms,Lux_Value,Temperature_C,Pressure_Pa\n");
        Serial.println("Created new log file with header.");
    }
    file.close();

    Serial.println("System ready. Starting data logging...");
}

void loop() {
    // The data reading and logging is now done inside logSensorData()
    logSensorData();

    // Wait for a period before the next cycle
    delay(2000); 
}
