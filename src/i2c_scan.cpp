/* i2c_scan.cpp — I2C bus scanner for ESP32
   Build via env:i2c_scan (does NOT interfere with main.cpp).
   Wiring: SDA=GPIO21  SCL=GPIO22  (default ESP32 pins)
   Scans all 127 addresses every 4 s and identifies common devices.       */

#ifdef ENV_I2C_SCAN   // guard: compiled only in the i2c_scan environment

#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 100000UL   // 100 kHz — safe for all devices

/* ── Known-device table ────────────────────────────────────────────────── */
struct KnownDevice { uint8_t addr; const char* name; };
static const KnownDevice KNOWN[] = {
    { 0x10, "VEML7700 (ambient light)" },
    { 0x18, "LIS3DH / LSM303 accel (SA0=0)" },
    { 0x19, "LIS3DH / LSM303 accel (SA0=1)" },
    { 0x1C, "MC3672 / LIS3MDL mag (SA0=0)" },
    { 0x1D, "MC3672 / LIS3MDL mag (SA0=1)" },
    { 0x1E, "HMC5883L / QMC5883 magnetometer" },
    { 0x20, "PCF8574 GPIO expander (A0-A2=0)" },
    { 0x21, "PCF8574 GPIO expander (A0=1)" },
    { 0x23, "BH1750 light (ADDR=VCC)" },
    { 0x27, "PCF8574 LCD backpack / GPIO expander" },
    { 0x29, "VL53L0X ToF / TSL2591 light" },
    { 0x38, "AHT10 / AHT20 / AHT21 temp+hum" },
    { 0x39, "TSL2561 light / APDS-9960 gesture" },
    { 0x3C, "SSD1306 / SSD1309 / SH1106 OLED (SA0=0)" },
    { 0x3D, "SSD1306 / SH1106 OLED (SA0=1)" },
    { 0x40, "INA219 / HDC1080 / HTU21D / SHT21" },
    { 0x41, "INA219 (A0=1) / SHT31" },
    { 0x44, "SHT30 / SHT31 temp+hum (ADDR=GND)" },
    { 0x45, "SHT30 / SHT31 temp+hum (ADDR=VCC)" },
    { 0x48, "ADS1115 / ADS1015 / TMP102 (A0-A1=GND)" },
    { 0x49, "ADS1115 / TMP102 (A0=VCC)" },
    { 0x4A, "ADS1115 (A0=SDA)" },
    { 0x4B, "ADS1115 (A0=SCL)" },
    { 0x50, "24Cxx EEPROM / AT24C32" },
    { 0x51, "24Cxx EEPROM" },
    { 0x53, "ADXL345 accelerometer (ADDR=GND)" },
    { 0x57, "MAX30102 pulse ox / AT24C32 (DS3231 board)" },
    { 0x5C, "BH1750 light (ADDR=GND)" },
    { 0x60, "Si5351 clock gen / MCP4725 DAC (A0=0)" },
    { 0x61, "MCP4725 DAC (A0=1)" },
    { 0x62, "SCD30 CO2 sensor" },
    { 0x63, "Si1145 UV/IR/vis light" },
    { 0x68, "DS3231 RTC / MPU-6050 / MPU-9250 (AD0=0)" },
    { 0x69, "MPU-6050 / MPU-9250 IMU (AD0=1)" },
    { 0x6A, "LSM6DS3 / ICM-42688 IMU (SA0=0)" },
    { 0x6B, "LSM6DS3 / ICM-42688 IMU (SA0=1)" },
    { 0x70, "TCA9548A I2C mux (A0-A2=0)" },
    { 0x71, "TCA9548A I2C mux (A0=1)" },
    { 0x72, "TCA9548A I2C mux (A1=1)" },
    { 0x76, "BME280 / BMP280 / BMP388 (SDO=0)" },
    { 0x77, "BME280 / BMP280 / BMP180 / MS5611 (SDO=1)" },
};
static const int KNOWN_COUNT = sizeof(KNOWN) / sizeof(KNOWN[0]);

static const char* identify(uint8_t addr) {
    for (int i = 0; i < KNOWN_COUNT; i++)
        if (KNOWN[i].addr == addr) return KNOWN[i].name;
    return "unknown device";
}

/* ── Read a single register from a device ──────────────────────────────── */
static uint8_t readReg(uint8_t devAddr, uint8_t reg) {
    Wire.beginTransmission(devAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    Wire.requestFrom(devAddr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

/* ── Probe known addresses to confirm chip identity ────────────────────── */
static void probeDevice(uint8_t addr) {
    switch (addr) {
        case 0x53: {
            /* ADXL345 DEVID register 0x00 → 0xE5 */
            uint8_t id = readReg(0x53, 0x00);
            if      (id == 0xE5) Serial.printf("         ↳ chip_id=0x%02X  → ADXL345 confirmed\n", id);
            else                 Serial.printf("         ↳ chip_id=0x%02X  → NOT ADXL345 (unknown at 0x53)\n", id);
            break;
        }
        case 0x62: {
            /* SCD30 firmware version command 0xD100, reply: major, minor, CRC */
            Wire.beginTransmission(0x62);
            Wire.write(0xD1); Wire.write(0x00);
            if (Wire.endTransmission() == 0) {
                delay(4);
                Wire.requestFrom((uint8_t)0x62, (uint8_t)3);
                if (Wire.available() >= 3) {
                    uint8_t maj = Wire.read(), min = Wire.read(); Wire.read();
                    Serial.printf("         ↳ SCD30 firmware v%d.%d confirmed\n", maj, min);
                } else {
                    Serial.println("         ↳ SCD30 firmware read failed (not SCD30?)");
                }
            } else {
                Serial.println("         ↳ SCD30 command failed");
            }
            break;
        }
        case 0x76:
        case 0x77: {
            /* BME/BMP register 0xD0 (chip_id) */
            uint8_t id = readReg(addr, 0xD0);
            const char* chip;
            switch (id) {
                case 0x60: chip = "BME280"; break;
                case 0x58: chip = "BMP280 (mass production)"; break;
                case 0x56:
                case 0x57: chip = "BMP280 (engineering sample)"; break;
                case 0x55: chip = "BMP180 / BMP085"; break;
                case 0x50: chip = "BMP388"; break;
                default:   chip = "unknown — may be MS5611 or other"; break;
            }
            Serial.printf("         ↳ chip_id=0x%02X  → %s\n", id, chip);
            break;
        }
    }
}

/* ── Scan ──────────────────────────────────────────────────────────────── */
static void scanBus() {
    Serial.println("\n══════════════════════════════════════════");
    Serial.printf(  "  I2C scan  SDA=GPIO%d  SCL=GPIO%d\n", SDA_PIN, SCL_PIN);
    Serial.println("══════════════════════════════════════════");

    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("  0x%02X (%3d)  →  %s\n", addr, addr, identify(addr));
            probeDevice(addr);
            found++;
        } else if (err == 4) {
            Serial.printf("  0x%02X (%3d)  →  ERROR (bus locked?)\n", addr, addr);
        }
    }

    if (found == 0)
        Serial.println("  No devices found — check wiring / pull-ups.");
    else
        Serial.printf("\n  Total: %d device(s) found.\n", found);

    Serial.println("══════════════════════════════════════════\n");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);
    Serial.println("ESP32 I2C Scanner ready.");
}

void loop() {
    scanBus();
    delay(4000);
}

#endif // ENV_I2C_SCAN
