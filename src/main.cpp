#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <climits>
#include <esp_sntp.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <DFRobot_ENS160.h>
#include <SensirionI2CScd4x.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_CCS811.h>
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>

#define LED_PIN             2
#define I2C_SDA             21
#define I2C_SCL             22
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

/* ── Sample buffer ─────────────────────────────────────────────────────────
   All sensor readings captured at sample time; RSSI at transmit time.
   Memory: 15 × 52 bytes = 780 bytes                                       */
struct Sample {
    float    aht_t,    aht_h;          // AHT20/21 — temperature °C, humidity %
    uint16_t ens_eco2, ens_tvoc;       // ENS160   — eCO₂ ppm, TVOC ppb
    uint8_t  ens_aqi;                  // ENS160   — AQI 1-5
    uint16_t scd41_co2;                // SCD41    — CO₂ ppm
    float    scd41_t,  scd41_h;        // SCD41    — temperature °C, humidity %
    float    bmp_t,    bmp_p;          // BMP280   — temperature °C, pressure hPa
    uint16_t ccs811_eco2, ccs811_tvoc; // CCS811   — eCO₂ ppm, TVOC ppb
    float    battery;
    unsigned long capturedAt_ms;
};
static Sample buf[MAX_SAMPLES];
static int    bufCount = 0;
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
static DFRobot_ENS160_I2C   ens160(&Wire, 0x53);
static SensirionI2CScd4x    scd4x;
static Adafruit_BMP280      bmp;
static Adafruit_CCS811      ccs811;
static bool aht_ok    = false;
static bool ens160_ok = false;
static bool scd4x_ok  = false;
static bool bmp_ok    = false;
static bool ccs811_ok = false;

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
    for (int n = 0; n < NUM_NETWORKS; n++) {
        Serial.printf("WiFi: trying %s ... ", networks[n].ssid);
        WiFi.begin(networks[n].ssid, networks[n].pass);
        unsigned long t = millis();
        while (WiFi.status() != WL_CONNECTED) {
            if (millis() - t > WIFI_TIMEOUT_MS) { Serial.println("timeout."); break; }
            delay(200);
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("OK  IP=%s  RSSI=%d dBm\n",
                          WiFi.localIP().toString().c_str(), WiFi.RSSI());
            return true;
        }
        WiFi.disconnect(true);
        delay(100);
    }
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
        snprintf(buf, sizeof(buf), "CO2 %uppm", s.scd41_co2);
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
        snprintf(buf, sizeof(buf), "Bat %.2fV   RSSI %d dBm", s.battery, WiFi.RSSI());
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

/* Publish all buffered samples.  syncTime / syncMillis anchor real time to a
   millis() value so each sample gets its own back-calculated Unix timestamp.
   ntpOffset (LONG_MIN = NTP failed) is included in every payload.           */
static void publishBuffer(time_t syncTime, unsigned long syncMillis, long ntpOffset,
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
        char payload[420];
        if (hasTime) {
            time_t sampleTs = syncTime -
                              (time_t)((syncMillis - buf[i].capturedAt_ms) / 1000);
            long off = (ntpOffset == LONG_MIN) ? -999999L : ntpOffset;
            snprintf(payload, sizeof(payload),
                "{\"ts\":%ld,"
                "\"aht_t\":%.1f,\"aht_h\":%.1f,"
                "\"ens_eco2\":%u,\"ens_tvoc\":%u,\"ens_aqi\":%u,"
                "\"ccs811_eco2\":%u,\"ccs811_tvoc\":%u,"
                "\"scd41_co2\":%u,\"scd41_t\":%.1f,\"scd41_h\":%.1f,"
                "\"bmp_t\":%.1f,\"bmp_p\":%.2f,"
                "\"battery\":%.1f,\"rssi\":%d,\"clock_offset\":%ld%s}",
                (long)sampleTs,
                buf[i].aht_t, buf[i].aht_h,
                buf[i].ens_eco2, buf[i].ens_tvoc, buf[i].ens_aqi,
                buf[i].ccs811_eco2, buf[i].ccs811_tvoc,
                buf[i].scd41_co2, buf[i].scd41_t, buf[i].scd41_h,
                buf[i].bmp_t, buf[i].bmp_p,
                buf[i].battery, rssi, off, refPart);
        } else {
            snprintf(payload, sizeof(payload),
                "{\"aht_t\":%.1f,\"aht_h\":%.1f,"
                "\"ens_eco2\":%u,\"ens_tvoc\":%u,\"ens_aqi\":%u,"
                "\"ccs811_eco2\":%u,\"ccs811_tvoc\":%u,"
                "\"scd41_co2\":%u,\"scd41_t\":%.1f,\"scd41_h\":%.1f,"
                "\"bmp_t\":%.1f,\"bmp_p\":%.2f,"
                "\"battery\":%.1f,\"rssi\":%d%s}",
                buf[i].aht_t, buf[i].aht_h,
                buf[i].ens_eco2, buf[i].ens_tvoc, buf[i].ens_aqi,
                buf[i].ccs811_eco2, buf[i].ccs811_tvoc,
                buf[i].scd41_co2, buf[i].scd41_t, buf[i].scd41_h,
                buf[i].bmp_t, buf[i].bmp_p,
                buf[i].battery, rssi, refPart);
        }
        bool ok = mqttClient.publish(MQTT_TOPIC, payload, /*retain=*/true);
        Serial.printf("  [%d/%d] %s  [%s]\n", i + 1, bufCount, payload,
                      ok ? "OK" : "FAIL");
        mqttClient.loop();
        delay(50);
    }
}

/* ── Timing state ──────────────────────────────────────────────────────── */
static unsigned long collectStart = 0;
static unsigned long lastSample   = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.printf("\nCPU: %u MHz  |  Chip temp: %.1f °C\n",
                  getCpuFrequencyMhz(), temperatureRead());
    Serial.printf("Cycle: collect %lus  |  sample every %lus  |  buffer %d slots\n",
                  COLLECT_DURATION_MS / 1000, SAMPLE_INTERVAL_MS / 1000, MAX_SAMPLES);

    /* ── I2C + sensors ────────────────────────────────────────────────── */
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    delay(2000);  // SCD30 needs up to 2 s after power-on before responding

    // Read raw status BEFORE init: AHT20 (factory-calibrated) → 0x18 (bits 3+4)
    //                                AHT10/AHT21 (needs cal cmd) → 0x08 (bit 3 only)
    Wire.requestFrom((uint8_t)AHTX0_I2CADDR_DEFAULT, (uint8_t)1);
    uint8_t ahtPreStatus = Wire.available() ? Wire.read() : 0xFF;
    const char* ahtType = ((ahtPreStatus & 0x18) == 0x18) ? "~AHT20" : "~AHT10/AHT21";

    aht_ok = aht.begin(&Wire);
    if (aht_ok) {
        Serial.printf("AHT20/21 (0x38): OK  pre_status=0x%02X(%s) post_status=0x%02X\n",
                      ahtPreStatus, ahtType, aht.getStatus());
    } else {
        Serial.printf("AHT20/21 (0x38): NOT FOUND  pre_status=0x%02X\n", ahtPreStatus);
    }
    delay(50);

    ens160_ok = (ens160.begin() == 0);
    if (ens160_ok) { ens160.setPWRMode(ENS160_STANDARD_MODE); }
    Serial.printf("ENS160   (0x53): %s\n", ens160_ok ? "OK" : "NOT FOUND");
    delay(50);

    scd4x.begin(Wire);
    scd4x.stopPeriodicMeasurement();   // no-op if idle; mandatory if left running
    delay(500);                        // SCD4x datasheet: 500 ms after stop
    uint16_t scd_err = scd4x.startPeriodicMeasurement();
    scd4x_ok = (scd_err == 0);
    if (scd_err) Serial.printf("SCD41    (0x62): NOT FOUND (err=0x%04X)\n", scd_err);
    else         Serial.println("SCD41    (0x62): OK");
    delay(50);

    bmp_ok = bmp.begin(0x77);
    Serial.printf("BMP280   (0x77): %s\n", bmp_ok ? "OK" : "NOT FOUND");

    // Probe CCS811 at both possible addresses before calling begin()
    Wire.beginTransmission(0x5A);
    bool ccs_at_5A = (Wire.endTransmission() == 0);
    Wire.beginTransmission(0x5B);
    bool ccs_at_5B = (Wire.endTransmission() == 0);
    uint8_t ccs_addr = ccs_at_5A ? 0x5A : 0x5B;
    if (ccs_at_5A || ccs_at_5B) {
        ccs811_ok = ccs811.begin(ccs_addr);
        if (ccs811_ok) { ccs811.setDriveMode(CCS811_DRIVE_MODE_10SEC); }
    }
    if (ccs811_ok) {
        Serial.printf("CCS811   (0x%02X): OK  probe=5A:%s 5B:%s  mode=10s  (warmup ~20min)\n",
                      ccs_addr,
                      ccs_at_5A ? "ACK" : "NACK",
                      ccs_at_5B ? "ACK" : "NACK");
    } else {
        Serial.printf("CCS811   (0x5A/0x5B): NOT FOUND  probe=5A:%s 5B:%s\n",
                      ccs_at_5A ? "ACK" : "NACK",
                      ccs_at_5B ? "ACK" : "NACK");
    }
    delay(50);

    /* Expand MQTT buffer to fit full JSON payload (420 data + MQTT overhead) */
    mqttClient.setBufferSize(640);

    /* Start with WiFi off */
    WiFi.mode(WIFI_OFF);
    /* ── e-Paper display init ──────────────────────────────────────── */
    SPI.begin(EPD_SCK, -1, EPD_MOSI, -1);
    display.init(115200, true, 50, false);
    display.setRotation(0);
    Serial.println("e-Paper: init OK");
    // Boot splash
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
    collectStart = millis();
    lastSample   = millis() - SAMPLE_INTERVAL_MS; // take first sample immediately
    Serial.println("── COLLECTING (WiFi OFF) ──────────────────────────────");
}

void loop() {
    unsigned long now = millis();

    /* ── 1. Collect sample ─────────────────────────────────────────────── */
    if (now - lastSample >= SAMPLE_INTERVAL_MS) {
        lastSample = now;
        if (bufCount < MAX_SAMPLES) {
            Sample s = {};
            s.capturedAt_ms = millis();

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

            /* SCD41 — CO₂ + temperature + humidity */
            if (scd4x_ok) {
                bool ready = false;
                scd4x.getDataReadyFlag(ready);
                if (ready) {
                    scd4x.readMeasurement(s.scd41_co2, s.scd41_t, s.scd41_h);
                }
            }

            /* BMP280 — temperature + pressure (no humidity) */
            if (bmp_ok) {
                s.bmp_t = bmp.readTemperature();
                s.bmp_p = bmp.readPressure() / 100.0f;   // Pa → hPa
            }

            /* CCS811 — eCO₂ + TVOC (T from BMP280 if available, H from AHT) */
            if (ccs811_ok) {
                if (aht_ok || bmp_ok) {
                    float comp_t = bmp_ok ? s.bmp_t : s.aht_t;
                    float comp_h = aht_ok ? s.aht_h : 50.0f;
                    ccs811.setEnvironmentalData(comp_h, comp_t);
                }
                if (ccs811.available()) {
                    if (!ccs811.readData()) {
                        s.ccs811_eco2 = ccs811.geteCO2();
                        s.ccs811_tvoc = ccs811.getTVOC();
                    } else {
                        Serial.println("CCS811: readData() error");
                    }
                }
            }

            buf[bufCount] = s;
            Serial.printf("Sample [%2d/%d]  aht=%.1f°C/%.0f%%  ens=eco2:%u/tvoc:%u/aqi:%u  ccs=eco2:%u/tvoc:%u  scd41=%uppm/%.1f°C/%.0f%%  bmp=%.1f°C/%.1fhPa  Δ(aht-bmp)=%+.1f  Δ(aht-scd)=%+.1f  (t+%lus)\n",
                          bufCount + 1, MAX_SAMPLES,
                          s.aht_t, s.aht_h,
                          s.ens_eco2, s.ens_tvoc, s.ens_aqi,
                          s.ccs811_eco2, s.ccs811_tvoc,
                          s.scd41_co2, s.scd41_t, s.scd41_h,
                          s.bmp_t, s.bmp_p,
                          s.aht_t - s.bmp_t,
                          s.aht_t - s.scd41_t,
                          (now - collectStart) / 1000);
            bufCount++;
        }
    }

    /* ── 2. Transmit window ────────────────────────────────────────────── */
    if (now - collectStart >= COLLECT_DURATION_MS) {
        Serial.println("── TRANSMITTING ───────────────────────────────────────");
        digitalWrite(LED_PIN, HIGH);

        if (wifiConnect()) {
            long ntpOffset        = syncNTP();
            WeatherRef ref        = fetchOpenMeteo();
            time_t syncTime       = time(nullptr);
            unsigned long syncMillis = millis();
            if (mqttConnect()) {
                publishBuffer(syncTime, syncMillis, ntpOffset, ref);
            } else {
                Serial.println("MQTT failed — samples discarded.");
            }
            // Update display regardless of MQTT result (last sample in buffer)
            if (bufCount > 0) {
                display.init(0, false, 50, false); // wake from hibernate, 50 ms RST
                updateDisplay(buf[bufCount - 1], ref, syncTime);
                display.hibernate();
            }
        } else {
            Serial.println("WiFi failed — samples discarded.");
        }

        wifiOff();
        digitalWrite(LED_PIN, LOW);

        /* Reset for next cycle */
        bufCount     = 0;
        collectStart = millis();
        lastSample   = millis() - SAMPLE_INTERVAL_MS; // take first sample immediately
        Serial.println("── COLLECTING (WiFi OFF) ──────────────────────────────");
    }
}