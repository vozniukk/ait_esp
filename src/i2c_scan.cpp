/* i2c_scan.cpp — I2C bus scanner for ESP32
   Build via env:i2c_scan (does NOT interfere with main.cpp).
   Bus 0 (Wire):  SDA=GPIO21  SCL=GPIO22  (default ESP32 pins)
   Bus 1 (Wire1): SDA=GPIO25  SCL=GPIO26
   Scans all 127 addresses on both buses every 4 s.                       */

#ifdef ENV_I2C_SCAN   // guard: compiled only in the i2c_scan environment

#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN  21
#define SCL_PIN  22
#define SDA2_PIN 25
#define SCL2_PIN 26
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
    { 0x36, "MAX17048 / MAX17049 fuel gauge" },
    { 0x60, "Si5351 clock gen / MCP4725 DAC (A0=0)" },
    { 0x61, "MCP4725 DAC (A0=1)" },
    { 0x62, "SCD30 / SCD41 CO2 sensor" },
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

/* ── Read a single register from a device on a given bus ───────────────── */
static uint8_t readReg(TwoWire& bus, uint8_t devAddr, uint8_t reg) {
    bus.beginTransmission(devAddr);
    bus.write(reg);
    if (bus.endTransmission(false) != 0) return 0xFF;
    bus.requestFrom(devAddr, (uint8_t)1);
    return bus.available() ? bus.read() : 0xFF;
}

/* ── Probe known addresses to confirm chip identity ────────────────────── */
static void probeDevice(TwoWire& bus, uint8_t addr) {
    switch (addr) {
        case 0x36: {
            /* MAX17048 VERSION register 0x08 → upper nibble 0x001x */
            bus.beginTransmission(0x36);
            bus.write(0x08);
            if (bus.endTransmission(false) == 0) {
                bus.requestFrom((uint8_t)0x36, (uint8_t)2);
                if (bus.available() >= 2) {
                    uint16_t ver = ((uint16_t)bus.read() << 8) | bus.read();
                    if      ((ver & 0xFFF0) == 0x0010)
                        Serial.printf("         \u21b3 version=0x%04X  \u2192 MAX17048 fuel gauge confirmed\n", ver);
                    else if ((ver & 0xFFF0) == 0x0020)
                        Serial.printf("         \u21b3 version=0x%04X  \u2192 MAX17049 fuel gauge\n", ver);
                    else
                        Serial.printf("         \u21b3 version=0x%04X  \u2192 MAX1704x-family fuel gauge\n", ver);
                }
            }
            break;
        }
        case 0x53: {
            /* ADXL345 DEVID register 0x00 → 0xE5 */
            uint8_t id = readReg(bus, 0x53, 0x00);
            if      (id == 0xE5) Serial.printf("         ↳ chip_id=0x%02X  → ADXL345 confirmed\n", id);
            else                 Serial.printf("         ↳ chip_id=0x%02X  → NOT ADXL345 (ENS160 or other at 0x53)\n", id);
            break;
        }
        case 0x5A:
        case 0x5B: {
            /* CCS811 HW_ID register 0x20 → 0x81 */
            uint8_t id = readReg(bus, addr, 0x20);
            if (id == 0x81) Serial.printf("         ↳ chip_id=0x%02X  → CCS811 confirmed\n", id);
            else            Serial.printf("         ↳ chip_id=0x%02X  → NOT CCS811 (unexpected)\n", id);
            break;
        }
        case 0x62: {
            /* SCD4x: stop periodic measurement first (in case it's still running) */
            bus.beginTransmission(0x62);
            bus.write(0x3F); bus.write(0x86);  // stop_periodic_measurement
            bus.endTransmission();
            delay(500);  // SCD41 requires 500 ms after stop

            /* SCD41 get_serial_number 0x3682, response 9 bytes */
            bus.beginTransmission(0x62);
            bus.write(0x36); bus.write(0x82);
            int err41 = bus.endTransmission();
            if (err41 == 0) {
                delay(10);
                uint8_t got = bus.requestFrom((uint8_t)0x62, (uint8_t)9);
                if (got >= 9) {
                    Serial.println("         ↳ SCD41 confirmed (serial number OK)");
                    break;
                }
                while (bus.available()) bus.read();
            }
            /* Fallback: SCD30 firmware-version 0xD100, response 3 bytes */
            bus.beginTransmission(0x62);
            bus.write(0xD1); bus.write(0x00);
            if (bus.endTransmission() == 0) {
                delay(10);
                bus.requestFrom((uint8_t)0x62, (uint8_t)3);
                if (bus.available() >= 3) {
                    uint8_t maj = bus.read(), min = bus.read(); bus.read();
                    Serial.printf("         ↳ SCD30 firmware v%d.%d confirmed\n", maj, min);
                } else {
                    Serial.println("         ↳ SCD30/SCD41 present — identify failed");
                }
            } else {
                Serial.printf("         ↳ SCD41 cmd err=%d; SCD30 fallback NAK'd\n", err41);
            }
            break;
        }
        case 0x76:
        case 0x77: {
            /* BME/BMP register 0xD0 (chip_id) */
            uint8_t id = readReg(bus, addr, 0xD0);
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

/* ── Scan one bus ───────────────────────────────────────────────────────── */
static void scanBus(TwoWire& bus, uint8_t sda, uint8_t scl) {
    Serial.println("\n══════════════════════════════════════════");
    Serial.printf(  "  I2C scan  SDA=GPIO%d  SCL=GPIO%d\n", sda, scl);
    Serial.println("══════════════════════════════════════════");

    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        bus.beginTransmission(addr);
        uint8_t err = bus.endTransmission();
        if (err == 0) {
            Serial.printf("  0x%02X (%3d)  →  %s\n", addr, addr, identify(addr));
            probeDevice(bus, addr);
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
    Wire1.begin(SDA2_PIN, SCL2_PIN, I2C_FREQ);
    Serial.println("ESP32 dual-bus I2C Scanner ready.");
}

void loop() {
    scanBus(Wire,  SDA_PIN,  SCL_PIN);
    scanBus(Wire1, SDA2_PIN, SCL2_PIN);
    delay(4000);
}

#endif // ENV_I2C_SCAN
