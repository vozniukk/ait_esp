# ePaper Air Monitor — Hardware Reference

ESP32-based indoor air quality monitor with e-paper display, MQTT telemetry, and dual I²C bus.

---

## Microcontroller

| Parameter | Value |
|-----------|-------|
| Board | ESP32 Dev Module (ESP32-D0WD-V3 rev 3.1) |
| CPU | Xtensa LX6 dual-core, 240 MHz max |
| Flash | 4 MB |
| RAM | 320 KB |
| MAC | `70:4b:ca:83:91:38` |
| Framework | Arduino (via PlatformIO) |
| CPU freq at runtime | 40 MHz during sensing, 80 MHz during WiFi |

---

## GPIO Pin Map

### I²C Bus 0 — Wire (sensors: AHT21, ENS160, SCD41)

| GPIO | Signal | Direction |
|------|--------|-----------|
| **21** | I2C SDA | Bidirectional |
| **22** | I2C SCL | Output |

Clock: 100 kHz (`Wire.setClock(100000)`)

### I²C Bus 1 — Wire1 (sensors: MAX17048, AHT21 #2, BMP280)

| GPIO | Signal | Direction |
|------|--------|-----------|
| **25** | I2C SDA | Bidirectional |
| **26** | I2C SCL | Output |

Clock: 100 kHz

### e-Paper Display — SPI (HSPI)

| GPIO | Signal | Direction | Notes |
|------|--------|-----------|-------|
| **5** | EPD_CS | Output | SPI chip select |
| **16** | EPD_SCK | Output | SPI clock |
| **23** | EPD_MOSI | Output | SPI data |
| **17** | EPD_DC | Output | Data / Command select |
| **15** | EPD_RST | Output | Hardware reset (active low) |
| **4** | EPD_BUSY | Input | HIGH = display busy |

MISO is not used (display is write-only).

### Miscellaneous

| GPIO | Signal | Direction | Notes |
|------|--------|-----------|-------|
| **2** | LED | Output | On-board blue LED |

### Reserved / Do Not Use

The following GPIOs are occupied and must **not** be reused:

`2`, `4`, `5`, `15`, `16`, `17`, `21`, `22`, `23`, `25`, `26`

---

## Sensors

### AHT21 — Temperature & Humidity (Bus 0)

| Parameter | Value |
|-----------|-------|
| I²C address | `0x38` |
| Bus | Wire (GPIO 21/22) |
| Library | `adafruit/Adafruit AHTX0 @ ^2.0.5` |
| Measurements | Temperature (°C), Relative Humidity (%) |
| Init | `aht.begin(&Wire)` |
| Operating voltage | 2.2–5.5 V |
| Temperature range | −40…+85 °C, ±0.3 °C |
| Humidity range | 0–100 %RH, ±2 %RH |
| Notes | Pre-status byte read before `begin()` to distinguish AHT10/AHT20/AHT21 variant |

### ENS160 — eCO₂, TVOC, AQI (Bus 0)

| Parameter | Value |
|-----------|-------|
| I²C address | `0x53` |
| Bus | Wire (GPIO 21/22) |
| Library | `dfrobot/DFRobot_ENS160 @ ^1.0.0` |
| Measurements | eCO₂ (ppm), TVOC (ppb), AQI (1–5) |
| Init | `ens160.begin()` → `ens160.setPWRMode(ENS160_STANDARD_MODE)` |
| Power mode | Standard (continuous) |
| Operating voltage | 1.71–1.89 V core / 1.8–3.6 V I/O |
| Chip ID | `0x60` (confirmed by scanner, not ADXL345) |
| Notes | Shares `0x53` with ADXL345 — chip ID distinguishes them |

### SCD41 — CO₂, Temperature & Humidity (Bus 0)

| Parameter | Value |
|-----------|-------|
| I²C address | `0x62` |
| Bus | Wire (GPIO 21/22) |
| Library | `sensirion/Sensirion I2C SCD4x @ ^0.4.0` |
| Measurements | CO₂ (ppm), Temperature (°C), Relative Humidity (%) |
| Measurement mode | **Low-power periodic** — one measurement every 30 s (~0.4 mA) |
| Init sequence | `scd4x.begin(Wire)` → `stopPeriodicMeasurement()` (500 ms wait) → `startLowPowerPeriodicMeasurement()` |
| Operating voltage | 2.4–5.5 V |
| CO₂ range | 400–5000 ppm, ±40 ppm ±5 % |
| Notes | Does **not** restart on deep-sleep wake; `begin(Wire)` only re-acquires the Wire handle. `stopPeriodicMeasurement` must be sent before any diagnostic command (e.g. `get_serial_number 0x3682`) |

### BMP280 — Atmospheric Pressure & Temperature (Bus 1)

| Parameter | Value |
|-----------|-------|
| I²C address | `0x77` (SDO = HIGH) |
| Bus | Wire1 (GPIO 25/26) |
| Library | `adafruit/Adafruit BMP280 Library @ ^2.6.0` |
| Measurements | Pressure (hPa), Temperature (°C) |
| Init | `bmp.begin(0x77)` |
| Sampling — temperature | `SAMPLING_X2` (×2 oversampling) |
| Sampling — pressure | `SAMPLING_X16` (×16 oversampling) |
| IIR filter | `FILTER_X16` |
| Standby time | `STANDBY_MS_500` |
| Operating mode | Sleep mode between reads (forced-mode trigger per sample) |
| Chip ID | `0x58` — BMP280 mass production confirmed |
| Operating voltage | 1.71–3.6 V |
| Pressure range | 300–1100 hPa, ±1 hPa |

### CCS811 — eCO₂ & TVOC (Bus 0)

| Parameter | Value |
|-----------|-------|
| I²C address | `0x5A` or `0x5B` (auto-detected at boot) |
| Bus | Wire (GPIO 21/22) |
| Library | `adafruit/Adafruit CCS811 Library @ ^1.1.0` |
| Measurements | eCO₂ (ppm), TVOC (ppb) |
| Drive mode | `CCS811_DRIVE_MODE_60SEC` — one measurement per 60 s |
| Init | `ccs811.begin(ccs_addr)` → `ccs811.setDriveMode(CCS811_DRIVE_MODE_60SEC)` |
| Operating voltage | 1.8–3.6 V |
| Warm-up | ~20 minutes until readings stabilise |
| HW_ID register | `0x20` → `0x81` (confirmed by scanner) |
| Notes | On deep-sleep **wake**, `begin()` is intentionally **skipped** (would send `SW_RESET` and clear drive mode). Raw I²C helpers read `STATUS (0x00)` and `ALG_RESULT_DATA (0x02)` directly. Environment compensation (temp + RH) written to `ENV_DATA (0x05)` using readings from AHT21. |

### MAX17048 — LiPo Battery Fuel Gauge (Bus 1)

| Parameter | Value |
|-----------|-------|
| I²C address | `0x36` |
| Bus | Wire1 (GPIO 25/26) |
| Library | None (raw I²C reads) |
| Measurements | Battery voltage (V), State of charge (%) |
| VERSION register | `0x08` → `0x0012` (MAX17048 confirmed by scanner) |
| Operating voltage | 2.7–5.5 V |

### AHT21 #2 — Temperature & Humidity (Bus 1)

| Parameter | Value |
|-----------|-------|
| I²C address | `0x38` |
| Bus | Wire1 (GPIO 25/26) |
| Notes | Second AHT sensor on the second I²C bus. Same model as Bus 0 sensor. Currently not integrated in `main.cpp` — presence confirmed by scanner. |

---

## e-Paper Display

| Parameter | Value |
|-----------|-------|
| Model | WeAct Studio 4.2" B/W/R |
| Part number | GDEY042Z98 |
| Controller | UC8176 |
| Resolution | 400 × 300 px |
| Colors | Black, White, Red |
| Interface | SPI (HSPI) |
| Library | `zinggjm/GxEPD2 @ ^1.6.2` |
| GxEPD2 class | `GxEPD2_3C<GxEPD2_420c_GDEY042Z98, …>` |
| Init | `SPI.begin(EPD_SCK, -1, EPD_MOSI, -1)` → `display.init(115200, true, 50, false)` |
| Rotation | 0° |
| Power after update | `display.hibernate()` |

### Display Layout (400 × 300 px)

```
┌──────────────────────┬─────────────────────┐
│  IN  temp (°C)       │  OUT temp (°C) [red]│  row 0–80
├──────────────────────┼─────────────────────┤
│  Pressure (hPa)      │  Humidity IN (%)    │  row 80–160
├──────────────────────┼─────────────────────┤
│  CO₂ (ppm)           │  TVOC (ppb)         │  row 160–240
├─────────────────────────────────────────────┤
│  Date/Time UTC   │  Battery V   RSSI dBm   │  row 240–300
└─────────────────────────────────────────────┘
```

Temperature source priority: SCD41 → BMP280 (fallback)  
Humidity source priority: SCD41 → AHT21 (fallback)  
CO₂: SCD41 | TVOC: ENS160  
Outdoor data: Open-Meteo API (lat 50.4354, lon 30.5059 — Kyiv)

---

## Power & Timing

| Parameter | Value |
|-----------|-------|
| Sample interval | 30 s (`SAMPLE_INTERVAL_MS`) |
| Collect window | 5 min / 300 s (`COLLECT_DURATION_MS`) |
| Buffer size | 15 samples (`MAX_SAMPLES`) |
| WiFi timeout | 15 s (`WIFI_TIMEOUT_MS`) |
| NTP sync timeout | 5 s (`NTP_TIMEOUT_MS`) |
| Open-Meteo timeout | 8 s (`OPENMETEO_TIMEOUT_MS`) |
| Sleep mechanism | `esp_deep_sleep` between samples |
| CPU — sensing | 40 MHz |
| CPU — WiFi/MQTT | 80 MHz |

---

## Communication

| Protocol | Role | Settings |
|----------|------|---------|
| I²C (Wire) | Sensors on Bus 0 | 100 kHz, GPIO 21/22 |
| I²C (Wire1) | Sensors on Bus 1 | 100 kHz, GPIO 25/26 |
| SPI (HSPI) | e-Paper display | GPIO 5/16/23/17/15/4 |
| WiFi | MQTT + NTP + HTTP | Station mode, 802.11 b/g/n |
| MQTT | Telemetry publish | Broker `130.61.100.186:1883`, topic `sensors/esp32`, retain=true |
| NTP | Time sync | `pool.ntp.org`, `time.nist.gov`, UTC |
| HTTP | Open-Meteo weather | `api.open-meteo.com` (plain HTTP) |
| Serial | Debug output | 115200 baud, USB-UART |

---

## Build Environments

| Environment | Purpose |
|-------------|---------|
| `env:auto` | Production — tries multiple WiFi networks automatically |
| `env:home` | Production — home network only |
| `env:office` | Production — office network only |
| `env:i2c_scan` | Diagnostic — dual-bus I²C scanner, no WiFi/MQTT |
