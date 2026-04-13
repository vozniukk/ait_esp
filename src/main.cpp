#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <climits>
#include <esp_sntp.h>
#include <esp_sleep.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <DFRobot_ENS160.h>
#include <SensirionI2CScd4x.h>
#include <Adafruit_BMP280.h>
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>

#define LED_PIN             2
#define I2C_SDA             21
#define I2C_SCL             22
#define I2C1_SDA            25
#define I2C1_SCL            26
// ── e-Paper SPI pins (WeAct 4.2" B/W/R via HSPI) ───────────────────────
#define EPD_BUSY    4
#define EPD_RST     15
#define EPD_DC      17
#define EPD_CS      5
#define EPD_SCK     16
#define EPD_MOSI    23
#define WIFI_TIMEOUT_MS     15000UL   // max time to wait for WiFi connection
#define SAMPLE_INTERVAL_MS  30000UL   // collect a sample every 30 s (WiFi OFF)
#define COLLECT_DURATION_MS 300000UL  // 5 minutes of collecting before transmit
#define MAX_SAMPLES         15        // 5min/30s = 10 samples; 15 is safe margin
#define NTP_SERVER1         "pool.ntp.org"
#define NTP_SERVER2         "time.nist.gov"
#define NTP_TIMEOUT_MS      5000UL    // max time to wait for NTP sync
#define OPENMETEO_TIMEOUT_MS 8000UL   // max time to wait for Open-Meteo HTTP response
#define UTC_OFFSET_SEC      0         // seconds east of UTC (0 = UTC, 3600 = UTC+1)

// ── SCD41 diagnostics mode ────────────────────────────────────────────────
// Define to run full SCD41 self-test + config dump on cold boot.
// Deep sleep is disabled while this is active so output stays visible.
#define SCD41_DIAG

/* ── Sample buffer ─────────────────────────────────────────────────────────
   All sensor readings captured at sample time; RSSI at transmit time.
   Stored in RTC slow memory so values survive deep sleep.                  */
struct Sample {
    float    aht_t,    aht_h;          // AHT20/21 — temperature °C, humidity %
    uint16_t ens_eco2, ens_tvoc;       // ENS160   — eCO₂ ppm, TVOC ppb
    uint8_t  ens_aqi;                  // ENS160   — AQI 1-5
    uint16_t scd41_co2;                // SCD41    — CO₂ ppm
    float    scd41_t,  scd41_h;        // SCD41    — temperature °C, humidity %
    float    bmp_t,    bmp_p;          // BMP280   — temperature °C, pressure hPa
    float    aht2_t, aht2_h;           // AHT21 #2 (Wire1) — temperature °C, humidity %
    float    battery_v;                   // MAX17048 — battery voltage V
    uint8_t  battery_pct;                 // MAX17048 — state of charge %
    unsigned long capturedAt_ms;
};
RTC_DATA_ATTR static Sample buf[MAX_SAMPLES];
RTC_DATA_ATTR static int    bufCount = 0;

/* MAX17048 — read battery voltage and state of charge over Wire1.
   VCELL register 0x02: 16-bit, 1 LSB = 78.125 µV → divide by 12800 to get V.
   SOC   register 0x04: upper byte = whole %, lower byte = 1/256 fraction.   */
static bool max17048_read(float &voltage_v, uint8_t &pct) {
    Wire1.beginTransmission(0x36);
    Wire1.write(0x02);  // VCELL
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((uint8_t)0x36, (uint8_t)2) < 2) return false;
    uint16_t raw_v = ((uint16_t)Wire1.read() << 8) | Wire1.read();
    voltage_v = (float)raw_v * 78.125e-6f;

    Wire1.beginTransmission(0x36);
    Wire1.write(0x04);  // SOC
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((uint8_t)0x36, (uint8_t)2) < 2) return false;
    pct = Wire1.read();  // whole-percent byte
    Wire1.read();        // discard fraction byte
    return true;
}
/* ── Outdoor weather reference (Open-Meteo, fetched once per WiFi cycle) ────── */
struct WeatherRef {
    float outdoor_t;   // °C  — temperature 2 m above ground
    float outdoor_p;   // hPa — surface pressure at location altitude
    float elevation;   // m   — terrain elevation returned by API
    float outdoor_rh;  // %   — relative humidity 2 m above ground
    float outdoor_ah;  // g/m³ — absolute humidity (computed)
    bool  ok;
};
static WeatherRef weatherRef = {};
/* ── WiFi credentials (injected by build_flags) ────────────────────────── */
struct WifiCred { const char* ssid; const char* pass; };
static const WifiCred networks[] = {
#if defined(WIFI_SSID_1)
    { WIFI_SSID_1, WIFI_PASS_1 },
#endif
#if defined(WIFI_SSID_2)
    { WIFI_SSID_2, WIFI_PASS_2 },
#endif
#if defined(WIFI_SSID_3)
    { WIFI_SSID_3, WIFI_PASS_3 },
#endif
#if defined(WIFI_SSID) && !defined(WIFI_SSID_1)
    { WIFI_SSID, WIFI_PASS },
#endif
};
static const int NUM_NETWORKS = sizeof(networks) / sizeof(networks[0]);

/* ── Sensors ───────────────────────────────────────────────────────────── */
static Adafruit_AHTX0       aht;
static Adafruit_AHTX0       aht2;
static DFRobot_ENS160_I2C   ens160(&Wire, 0x53);
static SensirionI2CScd4x    scd4x;
static Adafruit_BMP280      bmp(&Wire1);
static bool aht_ok    = false;
static bool aht2_ok   = false;
static bool ens160_ok = false;
static bool scd4x_ok  = false;
static bool bmp_ok    = false;

// WeAct 4.2" B/W/R = GDEY042Z98  400x300  controller UC8176
static GxEPD2_3C<GxEPD2_420c_GDEY042Z98, GxEPD2_420c_GDEY042Z98::HEIGHT>
    display(GxEPD2_420c_GDEY042Z98(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY));

/* ── MQTT client ───────────────────────────────────────────────────────── */
static WiFiClient   espClient;
static PubSubClient mqttClient(espClient);

/* Turn WiFi ON and connect to the first available known network.
   Returns true on success.                                                 */
static bool wifiConnect() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);   // WIFI_PS_NONE — radio stays active, APB held at 80 MHz after connect
    for (int n = 0; n < NUM_NETWORKS; n++) {
        WiFi.begin(networks[n].ssid, networks[n].pass);
        unsigned long t = millis();
        while (WiFi.status() != WL_CONNECTED) {
            if (millis() - t > WIFI_TIMEOUT_MS) { break; }
            delay(200);
        }
        if (WiFi.status() == WL_CONNECTED) {
            // WiFi driver now holds ESP_PM_APB_FREQ_MAX — APB is stable at 80 MHz.
            // Init Serial here instead of before wifiConnect() to avoid transient
            // APB drops that PM can cause during the delay() connect loop.
            Serial.begin(115200);
            Serial.printf("WiFi OK: %s  IP=%s  RSSI=%d dBm\n",
                          networks[n].ssid,
                          WiFi.localIP().toString().c_str(), WiFi.RSSI());
            return true;
        }
        WiFi.disconnect(true);
        delay(100);
    }
    // All networks failed.  setCpuFrequencyMhz(80) is still in effect (nothing
    // reverted it), so Serial.begin(115200) is valid here as well.
    Serial.begin(115200);
    Serial.println("WiFi: all networks failed.");
    return false;
}

/* Disconnect and power down the WiFi radio.                                */
static void wifiOff() {
    mqttClient.disconnect();
    espClient.stop();
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi OFF.");
}

/* Connect to MQTT broker. Returns true on success.                         */
static bool mqttConnect() {
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    String id = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.printf("MQTT: connecting as %s ... ", id.c_str());
    if (mqttClient.connect(id.c_str(), MQTT_USER, MQTT_PASS)) {
        Serial.println("OK");
        return true;
    }
    Serial.printf("FAIL rc=%d\n", mqttClient.state());
    return false;
}

/* Sync system clock via SNTP. Returns clock offset in seconds — how far the
   ESP's internal clock was off before sync (on fresh boot this equals the
   full Unix epoch distance, ~54 years; on a warm restart it shows real drift).
   Returns LONG_MIN on timeout. Call only while WiFi is connected.           */
static long syncNTP() {
    time_t pre_t        = time(nullptr);
    unsigned long pre_ms = millis();

    configTime(UTC_OFFSET_SEC, 0, NTP_SERVER1, NTP_SERVER2);

    struct tm tmInfo;
    if (!getLocalTime(&tmInfo, NTP_TIMEOUT_MS)) {
        Serial.println("NTP: timeout — timestamps unavailable.");
        return LONG_MIN;
    }

    unsigned long elapsed_ms = millis() - pre_ms;
    time_t post_t = time(nullptr);
    long offset = (long)post_t - (long)pre_t - (long)(elapsed_ms / 1000);

    sntp_sync_status_t status = sntp_get_sync_status();
    const char* syncStr = (status == SNTP_SYNC_STATUS_COMPLETED)   ? "fresh"
                        : (status == SNTP_SYNC_STATUS_IN_PROGRESS) ? "in_progress"
                        : "cached";
    Serial.printf("NTP: %s  %04d-%02d-%02d %02d:%02d:%02d UTC  clock_offset=%+lds\n",
                  syncStr,
                  tmInfo.tm_year + 1900, tmInfo.tm_mon + 1, tmInfo.tm_mday,
                  tmInfo.tm_hour, tmInfo.tm_min, tmInfo.tm_sec, offset);
    return offset;
}

/* Render last sample + outdoor reference on the e-Paper display.
   Layout (400×300 px):
     Row 1  (  0– 80)  IN temp  │  OUT temp
     Row 2  ( 80–160)  Pressure │  Humidity IN
     Row 3  (160–240)  CO2      │  TVOC
     Footer (240–300)  Date/time + Battery                                  */
static void updateDisplay(const Sample& s, const WeatherRef& ref, time_t ts) {
    display.setFullWindow();
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        char buf[40];

        const int16_t W    = 400;   // screen width
        const int16_t VDIV = 200;   // x of vertical divider
        const int16_t C2   = 206;   // x-start of right column

        // ── Row 1  Temperature ─────────────────────────────────────────
        float t_in = (s.scd41_t > -40.0f) ? s.scd41_t : s.bmp_t;
        display.setFont(&FreeSansBold18pt7b);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(4, 55);
        snprintf(buf, sizeof(buf), "IN  %.1fC", t_in);
        display.print(buf);
        if (ref.ok) {
            display.setTextColor(GxEPD_RED);
            display.setCursor(C2, 55);
            snprintf(buf, sizeof(buf), "OUT %.1fC", ref.outdoor_t);
            display.print(buf);
        }
        display.drawFastVLine(VDIV,   0, 80, GxEPD_BLACK);
        display.drawFastHLine(0,     80,  W, GxEPD_BLACK);

        // ── Row 2  Pressure | Humidity ─────────────────────────────────
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(4, 135);
        snprintf(buf, sizeof(buf), "%.1f hPa", s.bmp_p);
        display.print(buf);

        float hum_in = (s.scd41_h > 0.0f) ? s.scd41_h : s.aht_h;
        display.setCursor(C2, 135);
        snprintf(buf, sizeof(buf), "RH  %.0f%%", hum_in);
        display.print(buf);
        display.drawFastVLine(VDIV,  80, 80, GxEPD_BLACK);
        display.drawFastHLine(0,    160,  W, GxEPD_BLACK);

        // ── Row 3  CO₂ | TVOC ─────────────────────────────────────────
        display.setCursor(4, 215);
        if (s.scd41_co2 > 0)
            snprintf(buf, sizeof(buf), "CO2 %uppm", s.scd41_co2);
        else
            snprintf(buf, sizeof(buf), "CO2 ---");
        display.print(buf);

        display.setCursor(C2, 215);
        snprintf(buf, sizeof(buf), "TVOC %uppb", s.ens_tvoc);
        display.print(buf);
        display.drawFastVLine(VDIV, 160, 80, GxEPD_BLACK);
        display.drawFastHLine(0,   240,  W, GxEPD_BLACK);

        // ── Footer  Date/time + Battery ────────────────────────────────
        display.setFont(&FreeSansBold12pt7b);
        display.setTextColor(GxEPD_BLACK);
        display.setCursor(4, 265);
        if (ts > 1000000000L) {
            struct tm t;
            gmtime_r(&ts, &t);
            snprintf(buf, sizeof(buf), "%04d-%02d-%02d  %02d:%02d UTC",
                     t.tm_year+1900, t.tm_mon+1, t.tm_mday, t.tm_hour, t.tm_min);
            display.print(buf);
        }
        display.setFont(&FreeMonoBold9pt7b);
        display.setCursor(4, 285);
        snprintf(buf, sizeof(buf), "Bat %.2fV %u%%  RSSI %d dBm", s.battery_v, s.battery_pct, WiFi.RSSI());
        display.print(buf);

    } while (display.nextPage());
}

/* Query Open-Meteo for outdoor temperature, surface pressure and elevation.
   LOCATION_LAT / LOCATION_LON must be set via build_flags in platformio.ini.
   Returns WeatherRef; .ok == false on any error or missing coordinates.     */
static WeatherRef fetchOpenMeteo() {
    WeatherRef result = {};
#if !defined(LOCATION_LAT) || !defined(LOCATION_LON)
    Serial.println("OpenMeteo: LOCATION_LAT/LON not defined — skipping.");
    return result;
#else
    if ((float)LOCATION_LAT == 0.0f && (float)LOCATION_LON == 0.0f) {
        Serial.println("OpenMeteo: coordinates are 0,0 — set LOCATION_LAT/LON in platformio.ini.");
        return result;
    }
    char url[180];
    snprintf(url, sizeof(url),
        "http://api.open-meteo.com/v1/forecast"
        "?latitude=%.4f&longitude=%.4f"
        "&current=temperature_2m,surface_pressure",
        (float)LOCATION_LAT, (float)LOCATION_LON);

    WiFiClient plain;
    HTTPClient http;
    // Add relative_humidity_2m to current weather fields
    strncat(url, ",relative_humidity_2m", sizeof(url) - strlen(url) - 1);
    http.begin(plain, url);
    http.setTimeout(OPENMETEO_TIMEOUT_MS);
    Serial.printf("OpenMeteo: GET %s\n", url);
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        Serial.printf("OpenMeteo: HTTP error %d\n", code);
        http.end();
        return result;
    }

    // Read full body into String first — stream parsing is unreliable on ESP32
    String body = http.getString();
    http.end();
    Serial.printf("OpenMeteo: body_len=%d\n", body.length());

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
        Serial.printf("OpenMeteo: JSON parse error: %s  body=%s\n",
                      err.c_str(), body.substring(0, 80).c_str());
        return result;
    }

    result.elevation  = doc["elevation"].as<float>();
    result.outdoor_t  = doc["current"]["temperature_2m"].as<float>();
    result.outdoor_p  = doc["current"]["surface_pressure"].as<float>();
    result.outdoor_rh = doc["current"]["relative_humidity_2m"].as<float>();
    // Absolute humidity g/m³ via Magnus formula: AH = 216.7 * (rh/100 * 6.112 * exp(17.67*T/(T+243.5))) / (273.15+T)
    {
        float T = result.outdoor_t, rh = result.outdoor_rh;
        float es = 6.112f * expf(17.67f * T / (T + 243.5f));
        result.outdoor_ah = 216.7f * (rh / 100.0f * es) / (273.15f + T);
    }
    result.ok         = true;
    Serial.printf("OpenMeteo: OK  outdoor=%.1f°C  rh=%.0f%%  ah=%.1fg/m3  pressure=%.1fhPa  elevation=%.0fm\n",
                  result.outdoor_t, result.outdoor_rh, result.outdoor_ah, result.outdoor_p, result.elevation);
    return result;
#endif
}

/* Publish all buffered samples.  syncTime anchors real time; each sample's
   timestamp is derived from its index (deep-sleep safe, no millis() drift).
   ntpOffset (LONG_MIN = NTP failed) is included in every payload.           */
static void publishBuffer(time_t syncTime, long ntpOffset,
                          const WeatherRef& ref) {
    int  rssi    = WiFi.RSSI();
    bool hasTime = (syncTime > 1000000000L);   // sanity: past year 2001
    Serial.printf("Publishing %d sample(s)  hasTime=%s ...\n",
                  bufCount, hasTime ? "yes" : "no");

    // Build optional Open-Meteo suffix once (same for all samples in a cycle)
    char refPart[100] = "";
    if (ref.ok) {
        snprintf(refPart, sizeof(refPart),
            ",\"ref_t\":%.1f,\"ref_rh\":%.0f,\"ref_ah\":%.1f,\"ref_p\":%.1f,\"ref_elev\":%.0f",
            ref.outdoor_t, ref.outdoor_rh, ref.outdoor_ah, ref.outdoor_p, ref.elevation);
    }

    for (int i = 0; i < bufCount; i++) {
        char payload[480];
        if (hasTime) {
            // Sample i was taken (bufCount-1-i) intervals before transmit.
            // Index-based calculation works correctly across deep-sleep wakes.
            time_t sampleTs = syncTime -
                              (time_t)((bufCount - 1 - i) * (SAMPLE_INTERVAL_MS / 1000));
            long off = (ntpOffset == LONG_MIN) ? -999999L : ntpOffset;
            snprintf(payload, sizeof(payload),
                "{\"ts\":%ld,"
                "\"aht_t\":%.1f,\"aht_h\":%.1f,"
                "\"aht2_t\":%.1f,\"aht2_h\":%.1f,"
                "\"ens_eco2\":%u,\"ens_tvoc\":%u,\"ens_aqi\":%u,"
                "\"scd41_co2\":%u,\"scd41_t\":%.1f,\"scd41_h\":%.1f,"
                "\"bmp_t\":%.1f,\"bmp_p\":%.2f,"
                "\"battery_v\":%.2f,\"battery_pct\":%u,"
                "\"rssi\":%d,\"clock_offset\":%ld%s}",
                (long)sampleTs,
                buf[i].aht_t, buf[i].aht_h,
                buf[i].aht2_t, buf[i].aht2_h,
                buf[i].ens_eco2, buf[i].ens_tvoc, buf[i].ens_aqi,
                buf[i].scd41_co2, buf[i].scd41_t, buf[i].scd41_h,
                buf[i].bmp_t, buf[i].bmp_p,
                buf[i].battery_v, buf[i].battery_pct,
                rssi, off, refPart);
        } else {
            snprintf(payload, sizeof(payload),
                "{\"aht_t\":%.1f,\"aht_h\":%.1f,"
                "\"aht2_t\":%.1f,\"aht2_h\":%.1f,"
                "\"ens_eco2\":%u,\"ens_tvoc\":%u,\"ens_aqi\":%u,"
                "\"scd41_co2\":%u,\"scd41_t\":%.1f,\"scd41_h\":%.1f,"
                "\"bmp_t\":%.1f,\"bmp_p\":%.2f,"
                "\"battery_v\":%.2f,\"battery_pct\":%u,"
                "\"rssi\":%d%s}",
                buf[i].aht_t, buf[i].aht_h,
                buf[i].aht2_t, buf[i].aht2_h,
                buf[i].ens_eco2, buf[i].ens_tvoc, buf[i].ens_aqi,
                buf[i].scd41_co2, buf[i].scd41_t, buf[i].scd41_h,
                buf[i].bmp_t, buf[i].bmp_p,
                buf[i].battery_v, buf[i].battery_pct,
                rssi, refPart);
        }
        bool ok = mqttClient.publish(MQTT_TOPIC, payload, /*retain=*/true);
        Serial.printf("  [%d/%d] %s  [%s]\n", i + 1, bufCount, payload,
                      ok ? "OK" : "FAIL");
        mqttClient.loop();
        delay(50);
    }
}

/* ── Timing state ──────────────────────────────────────────────────────── */
// Replaced by RTC_DATA_ATTR bufCount and deep-sleep timer wakeups.

void setup() {
    // 40 MHz during sensing saves ~30 mA vs default 240 MHz; raised to 80 MHz for WiFi.
    setCpuFrequencyMhz(40);
    Serial.begin(115200);
    delay(100);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    WiFi.mode(WIFI_OFF);

    bool coldBoot = (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER);
    unsigned long wakeStart = millis();


    if (coldBoot) {
        bufCount = 0;   // RTC memory is 0 on power-on, but be explicit
        Serial.printf("\nCOLD BOOT  CPU=%uMHz  chip_temp=%.1f°C\n",
                      getCpuFrequencyMhz(), temperatureRead());
        Serial.printf("Cycle: collect %lus | sample every %lus | buffer %d slots\n",
                      COLLECT_DURATION_MS / 1000, SAMPLE_INTERVAL_MS / 1000, MAX_SAMPLES);
    } else {
        Serial.printf("\nWAKE [%d/%d]  CPU=%uMHz\n",
                      bufCount + 1, MAX_SAMPLES, getCpuFrequencyMhz());
    }

    /* ── I2C buses ───────────────────────────────────────────────────── */
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    Wire1.begin(I2C1_SDA, I2C1_SCL, 100000);

    if (coldBoot) {
        delay(2000);  // sensors need up to 2 s after power-on before responding

        // Read raw status BEFORE init: AHT20 (factory-calibrated) → 0x18 (bits 3+4)
        //                                AHT10/AHT21 (needs cal cmd) → 0x08 (bit 3 only)
        Wire.requestFrom((uint8_t)AHTX0_I2CADDR_DEFAULT, (uint8_t)1);
        uint8_t ahtPreStatus = Wire.available() ? Wire.read() : 0xFF;
        const char* ahtType = ((ahtPreStatus & 0x18) == 0x18) ? "~AHT20" : "~AHT10/AHT21";

        aht_ok = aht.begin(&Wire);
        if (aht_ok)
            Serial.printf("AHT20/21 (0x38): OK  pre_status=0x%02X(%s) post_status=0x%02X\n",
                          ahtPreStatus, ahtType, aht.getStatus());
        else
            Serial.printf("AHT20/21 (0x38): NOT FOUND  pre_status=0x%02X\n", ahtPreStatus);
        delay(50);

        ens160_ok = (ens160.begin() == 0);
        if (ens160_ok) { ens160.setPWRMode(ENS160_STANDARD_MODE); }
        Serial.printf("ENS160   (0x53): %s\n", ens160_ok ? "OK" : "NOT FOUND");
        delay(50);

        scd4x.begin(Wire);
        scd4x.stopPeriodicMeasurement();   // safety: stop if left running
        delay(500);                        // SCD4x datasheet: 500 ms after stop
        { uint16_t ri = scd4x.reinit();   // reload factory calibration from NVM
          delay(30);
          Serial.printf("SCD41    reinit: %s (0x%04X)\n", ri == 0 ? "OK" : "ERR", ri); }

#ifdef SCD41_DIAG
        // ── Full diagnostics (cold boot only) ─────────────────────────
        Serial.println("\n══ SCD41 DIAGNOSTICS ══════════════════════════");

        // Serial number
        { uint16_t s0, s1, s2;
          uint16_t e = scd4x.getSerialNumber(s0, s1, s2);
          if (e == 0) Serial.printf("  Serial:   0x%04X%04X%04X\n", s0, s1, s2);
          else        Serial.printf("  Serial:   ERR 0x%04X\n", e); }

        // ASC enabled?
        { uint16_t asc;
          uint16_t e = scd4x.getAutomaticSelfCalibration(asc);
          if (e == 0) Serial.printf("  ASC:      %s\n", asc ? "ENABLED" : "DISABLED");
          else        Serial.printf("  ASC:      ERR 0x%04X\n", e); }

        // Temperature offset
        { float tOff;
          uint16_t e = scd4x.getTemperatureOffset(tOff);
          if (e == 0) Serial.printf("  T-offset: %.2f degC\n", tOff);
          else        Serial.printf("  T-offset: ERR 0x%04X\n", e); }

        // Sensor altitude
        { uint16_t alt;
          uint16_t e = scd4x.getSensorAltitude(alt);
          if (e == 0) Serial.printf("  Altitude: %u m\n", alt);
          else        Serial.printf("  Altitude: ERR 0x%04X\n", e); }

        // Self-test (takes ~10 s)
        { Serial.println("  Self-test: running (~10s)...");
          uint16_t status;
          uint16_t e = scd4x.performSelfTest(status);
          if (e != 0)       Serial.printf("  Self-test: I2C ERR 0x%04X\n", e);
          else if (status)  Serial.printf("  Self-test: FAIL status=0x%04X\n", status);
          else              Serial.println("  Self-test: PASS"); }

        // Continuous read for 10 cycles (50 s) — CO2 should appear by cycle 3
        Serial.println("  Starting periodic-5s; reading 10 cycles:");
        scd4x_ok = (scd4x.startPeriodicMeasurement() == 0);
        Serial.printf("  startPeriodicMeasurement: %s\n", scd4x_ok ? "OK" : "FAIL");
        for (int ci = 0; ci < 10 && scd4x_ok; ci++) {
            delay(5100);
            bool ready = false;
            scd4x.getDataReadyFlag(ready);
            if (ready) {
                uint16_t co2; float t, h;
                uint16_t err = scd4x.readMeasurement(co2, t, h);
                if (err) Serial.printf("  [%2d] readMeasurement ERR 0x%04X\n", ci+1, err);
                else     Serial.printf("  [%2d] CO2=%uppm  T=%.1fC  RH=%.0f%%\n", ci+1, co2, t, h);
            } else {
                Serial.printf("  [%2d] NOT READY\n", ci+1);
            }
        }
        Serial.println("══ END DIAGNOSTICS ═══════════════════════════");
        // Keep periodic mode running for normal operation below
        if (!scd4x_ok) { scd4x_ok = (scd4x.startPeriodicMeasurement() == 0); }
#else
        // Periodic mode (5 s cycle): sensor keeps measuring during deep sleep.
        // After 30 s of sleep, data is always ready on wake; CO2 algorithm
        // accumulates ≥2 measurements before the first sample is read.
        scd4x_ok = (scd4x.startPeriodicMeasurement() == 0);
        if (!scd4x_ok) Serial.println("SCD41    (0x62): NOT FOUND");
        else           Serial.println("SCD41    (0x62): OK  mode=periodic-5s");
#endif
        delay(50);

        bmp_ok = bmp.begin(0x77);
        if (bmp_ok) {
            bmp.setSampling(Adafruit_BMP280::MODE_SLEEP,        // start in sleep
                            Adafruit_BMP280::SAMPLING_X2,       // temperature
                            Adafruit_BMP280::SAMPLING_X16,      // pressure
                            Adafruit_BMP280::FILTER_X16,
                            Adafruit_BMP280::STANDBY_MS_500);
        }
        Serial.printf("BMP280   (0x77/Wire1): %s%s\n", bmp_ok ? "OK" : "NOT FOUND",
                      bmp_ok ? "  mode=forced" : "");

        aht2_ok = aht2.begin(&Wire1);
        Serial.printf("AHT21 #2 (0x38/Wire1): %s\n", aht2_ok ? "OK" : "NOT FOUND");
        delay(50);

        /* Expand MQTT buffer to fit full JSON payload */
        mqttClient.setBufferSize(640);

        /* e-Paper boot splash */
        SPI.begin(EPD_SCK, -1, EPD_MOSI, -1);
        display.init(115200, true, 50, false);
        display.setRotation(0);
        Serial.println("e-Paper: init OK");
        display.setFullWindow();
        display.firstPage();
        do {
            display.fillScreen(GxEPD_WHITE);
            display.setFont(&FreeSansBold12pt7b);
            display.setTextColor(GxEPD_RED);
            display.setCursor(4, 30);
            display.print("ESP32 Air Monitor");
            display.setFont(&FreeMonoBold9pt7b);
            display.setTextColor(GxEPD_BLACK);
            display.setCursor(4, 56);
            display.print("Booting... collecting data");
            display.setCursor(4, 76);
            display.print("First update in ~5 min");
        } while (display.nextPage());
        display.hibernate();
        Serial.println("e-Paper: boot splash done");

    } else {
        /* ── Wake from deep sleep ────────────────────────────────────────
           Sensors stayed powered during deep sleep; only re-init ESP-side
           library handles. Do NOT restart SCD41 periodic measurement.     */
        aht_ok    = aht.begin(&Wire);
        ens160_ok = (ens160.begin() == 0);
        if (ens160_ok) { ens160.setPWRMode(ENS160_STANDARD_MODE); }

        // SCD41: re-acquire Wire handle. Sensor stayed in periodic-5s mode during sleep.
        // Do NOT issue a new measurement command; just read the result below.
        scd4x.begin(Wire);
        { bool _r = false; scd4x_ok = (scd4x.getDataReadyFlag(_r) == 0); }

        bmp_ok = bmp.begin(0x77);
        if (bmp_ok) {
            bmp.setSampling(Adafruit_BMP280::MODE_SLEEP,
                            Adafruit_BMP280::SAMPLING_X2,
                            Adafruit_BMP280::SAMPLING_X16,
                            Adafruit_BMP280::FILTER_X16,
                            Adafruit_BMP280::STANDBY_MS_500);
        }

        aht2_ok = aht2.begin(&Wire1);
    }

    /* ── Take sample ──────────────────────────────────────────────────── */
    if (bufCount < MAX_SAMPLES) {
        Sample s = {};
        s.capturedAt_ms = millis();     // local uptime within this wake only

        /* AHT20/21 — temperature + humidity */
        if (aht_ok) {
            sensors_event_t aht_hev, aht_tev;
            if (aht.getEvent(&aht_hev, &aht_tev)) {
                s.aht_t = aht_tev.temperature;
                s.aht_h = aht_hev.relative_humidity;
            }
        }

        /* ENS160 — eCO₂, TVOC, AQI (T+H compensation from AHT) */
        if (ens160_ok) {
            if (aht_ok) { ens160.setTempAndHum(s.aht_t, s.aht_h); }
            if (ens160.getENS160Status() == 0) {
                s.ens_eco2 = ens160.getECO2();
                s.ens_tvoc = ens160.getTVOC();
                s.ens_aqi  = ens160.getAQI();
            } else {
                s.ens_eco2 = 0;
                s.ens_tvoc = 0;
                s.ens_aqi  = 0;
            }
        }

        /* SCD41 — CO₂ + temperature + humidity (periodic 5-s measurement).
           Sensor runs continuously during deep sleep. On wake: data is ready
           immediately. On cold boot: e-paper init (~12 s) covers ≥2 cycles. */
        if (scd4x_ok) {
            bool ready = false;
            scd4x.getDataReadyFlag(ready);
            if (!ready) {
                // Poll up to 6 s (cold boot edge case: still in first cycle)
                unsigned long t0 = millis();
                while (!ready && millis() - t0 < 6000) {
                    delay(200);
                    scd4x.getDataReadyFlag(ready);
                }
            }
            if (ready) {
                uint16_t scd_err = scd4x.readMeasurement(s.scd41_co2, s.scd41_t, s.scd41_h);
                if (scd_err)
                    Serial.printf("SCD41 readMeasurement err=0x%04X\n", scd_err);
            } else {
                Serial.println("SCD41: NOT ready after 6s poll");
            }
        }

        /* BMP280 — temperature + pressure (forced measurement, then back to sleep) */
        if (bmp_ok) {
            bmp.takeForcedMeasurement();               // triggers + waits ~40 ms
            s.bmp_t = bmp.readTemperature();
            s.bmp_p = bmp.readPressure() / 100.0f;   // Pa → hPa
        }

        /* AHT21 #2 — temperature + humidity from Wire1 */
        if (aht2_ok) {
            sensors_event_t aht2_hev, aht2_tev;
            if (aht2.getEvent(&aht2_hev, &aht2_tev)) {
                s.aht2_t = aht2_tev.temperature;
                s.aht2_h = aht2_hev.relative_humidity;
            }
        }

        /* MAX17048 — battery voltage and state of charge */
        {
            float bv = 0.0f; uint8_t pct = 0;
            if (max17048_read(bv, pct)) {
                s.battery_v   = bv;
                s.battery_pct = pct;
            }
        }

        buf[bufCount] = s;
        Serial.printf("Sample [%2d/%d]  aht=%.1f°C/%.0f%%  aht2=%.1f°C/%.0f%%"
                      "  ens=eco2:%u/tvoc:%u/aqi:%u"
                      "  scd41=%uppm/%.1f°C/%.0f%%"
                      "  bmp=%.1f°C/%.1fhPa  bat=%.2fV/%u%%"
                      "  Δ(aht-bmp)=%+.1f  Δ(aht-scd)=%+.1f\n",
                      bufCount + 1, MAX_SAMPLES,
                      s.aht_t, s.aht_h, s.aht2_t, s.aht2_h,
                      s.ens_eco2, s.ens_tvoc, s.ens_aqi,
                      s.scd41_co2, s.scd41_t, s.scd41_h,
                      s.bmp_t, s.bmp_p, s.battery_v, s.battery_pct,
                      s.aht_t - s.bmp_t,
                      s.aht_t - s.scd41_t);
        bufCount++;
    }

    /* ── Transmit when collect cycle is complete ─────────────────────── */
    bool cycleComplete = ((uint32_t)bufCount * SAMPLE_INTERVAL_MS >= COLLECT_DURATION_MS)
                       || (bufCount >= MAX_SAMPLES);
    if (cycleComplete) {
        Serial.println("── TRANSMITTING ───────────────────────────────────────");
        Serial.flush();              // drain TX FIFO before APB clock changes
        digitalWrite(LED_PIN, HIGH);
        setCpuFrequencyMhz(80);      // WiFi requires ≥ 80 MHz
        // Serial.begin is intentionally deferred to inside wifiConnect() —
        // called only after WiFi connects and the WiFi driver holds the APB
        // lock, guaranteeing a stable 80 MHz when the UART divider is set.
        mqttClient.setBufferSize(640);

        if (wifiConnect()) {
            long ntpOffset  = syncNTP();
            WeatherRef ref  = fetchOpenMeteo();
            time_t syncTime = time(nullptr);
            if (mqttConnect()) {
                publishBuffer(syncTime, ntpOffset, ref);
            } else {
                Serial.println("MQTT failed — samples discarded.");
            }
            if (bufCount > 0) {
                SPI.begin(EPD_SCK, -1, EPD_MOSI, -1);
                display.init(0, false, 50, false);  // wake from hibernate, 50 ms RST
                updateDisplay(buf[bufCount - 1], ref, syncTime);
                display.hibernate();
            }
        } else {
            Serial.println("WiFi failed — samples discarded.");
        }

        wifiOff();
        digitalWrite(LED_PIN, LOW);
        bufCount = 0;
        Serial.flush();              // drain TX FIFO before APB clock changes
        setCpuFrequencyMhz(40);
        Serial.begin(115200);          // APB clock changed — resync UART baud divider
        Serial.println("── COLLECTING (deep sleep) ────────────────────────────");
    }

    /* ── Deep sleep until next sample interval ───────────────────────── */
#ifdef SCD41_DIAG
    Serial.println("[DIAG] Deep sleep DISABLED — reset ESP to restart.");
    Serial.flush();
    while (true) { delay(1000); }   // halt; inspect output, then reset manually
#endif
    unsigned long elapsed = millis() - wakeStart;
    uint64_t sleepUs = (SAMPLE_INTERVAL_MS > elapsed)
                       ? (uint64_t)(SAMPLE_INTERVAL_MS - elapsed) * 1000ULL
                       : 100000ULL;    // 100 ms minimum if processing overran
    Serial.printf("Awake %lu ms → sleeping %llu ms.\n", elapsed, sleepUs / 1000ULL);
    Serial.flush();
    esp_sleep_enable_timer_wakeup(sleepUs);
    esp_deep_sleep_start();
}

void loop() {
    // Never reached — setup() always ends with esp_deep_sleep_start().
    esp_deep_sleep_start();
}