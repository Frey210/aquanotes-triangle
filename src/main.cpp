/*
 * Aerasea RTOS UI – ESP32-S3
 * Versi: 4 tombol (UP, DOWN, OK, BACK), tanpa encoder
 * Display: TFT ILI9341 320x240 (SPI)
 * PlatformIO Version
 */

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WiFiManager.h>  
#include <time.h>
#include <math.h>
#include "ui_types.h"
#include "aerasea_logo_128x64_inverted.h"

// =======================
//  KONFIG PERANGKAT & NET
// =======================

// TFT ILI9341 (SPI) - landscape
static const uint8_t TFT_ROTATION = 1;

// TFT pin mapping (ESP32-S3)
#define TFT_CS   10   // Chip Select
#define TFT_DC   9    // Data/Command
#define TFT_RST  8    // Reset (connect to 3.3V if LCD has reset pin)
#define TFT_MOSI 11
#define TFT_MISO 13
#define TFT_SCK  12

// 16-bit color aliases (RGB565)
#define TFT_BLACK     0x0000
#define TFT_NAVY      0x000F
#define TFT_DARKGREY  0x7BEF
#define TFT_LIGHTGREY 0xC618
#define TFT_WHITE     0xFFFF
#define TFT_ORANGE    0xFD20
#define TFT_YELLOW    0xFFE0
#define TFT_CYAN      0x07FF
#define TFT_GREEN     0x07E0
#define TFT_BLUE      0x001F
#define TFT_MAGENTA   0xF81F

// DS18B20
#define DS18B20_PIN 18

// RGB LED (common cathode)
#define LED_R_PIN 2
#define LED_G_PIN 3
#define LED_B_PIN 4

// LEDC (PWM) channels
#define LEDC_CH_R 0
#define LEDC_CH_G 1
#define LEDC_CH_B 2
#define LEDC_FREQ 5000
#define LEDC_RES  8

// RS485 (shared bus)
#define RS485_RX_PIN    16
#define RS485_TX_PIN    15
#define RS485_DE_RE_PIN 14

// Slave IDs (pH/EC/NH4 sekarang unik, DO tetap 55)
#define RK_SLAVE_ID   0x06   // RK500-09 multi-parameter (default Modbus addr 06H)
#define PH_SLAVE_ID   12   // pH
#define EC_SLAVE_ID   30   // EC/TDS/Sal
#define NH4_SLAVE_ID  1   // NH4
#define DO_SLAVE_ID   55  // DO (default)

// Serial format
#define PH_BAUD   9600
#define EC_BAUD   9600
#define NH4_BAUD  9600
#define DO_BAUD   9600

// Server & Identitas
const char* POST_URL   = "https://aeraseaku.inkubasistartupunhas.id/sensor/";
const char* UID        = "AER2023AQ0020";
const char* FW_VERSION = "v1.3.0-RTOS-BTN4";
const uint32_t POST_INTERVAL_MS = 10000; // 10 detik
const uint32_t HTTP_TIMEOUT_MS  = 3500;

// WiFi Manager & NTP
// Beberapa server NTP untuk fallback
const char* NTP_SERVER   = "pool.ntp.org";      // utama
const char* NTP_SERVER_2 = "time.google.com";   // fallback 1
const char* NTP_SERVER_3 = "id.pool.ntp.org";   // fallback 2 (regional)

const long  GMT_OFFSET   = 8 * 3600; // GMT+8
const int   DAYLIGHT_OFF = 0;


// =======================
//  INPUT: 4 BUTTON
// =======================
// Button aktif LOW (INPUT_PULLUP)
#define BTN_UP     5
#define BTN_DOWN   6
#define BTN_OK     7
#define BTN_BACK   17

// =======================
//  OBJEK GLOBAL
// =======================
class TFTWrapper : public Adafruit_ILI9341 {
public:
  using Adafruit_ILI9341::Adafruit_ILI9341;

  void setTextFont(uint8_t size) {
    setTextSize(size);
    setFont(NULL);
  }

  int16_t textWidth(const char* text, uint8_t size) {
    setTextSize(size);
    int16_t x1, y1;
    uint16_t w, h;
    getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    return static_cast<int16_t>(w);
  }

  void drawString(const char* text, int16_t x, int16_t y) {
    setCursor(x, y);
    print(text);
  }

  void drawCentreString(const char* text, int16_t x, int16_t y, uint8_t size) {
    setTextSize(size);
    int16_t x1, y1;
    uint16_t w, h;
    getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    setCursor(x - static_cast<int16_t>(w / 2), y);
    print(text);
  }
};

TFTWrapper tft(TFT_CS, TFT_DC, TFT_RST);

OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
HardwareSerial RS485(2);

// Multi-parameter sensor RK500-09
ModbusMaster mb_rk;

// (opsional) legacy objects untuk menu kalibrasi lama
ModbusMaster mb_ph, mb_ec, mb_nh4, mb_do;

struct RuntimeStatus {
  char postStatus[16] = "---";
};

struct AsyncDS18B20 {
  explicit AsyncDS18B20(DallasTemperature& sensor) : sensor(sensor) {}

  void begin(){
    sensor.begin();
    sensor.setResolution(12);
    sensor.setWaitForConversion(false); // non-blocking conversion
    lastRequestMs = 0;
  }

  void tick(){
    uint32_t now = millis();
    if (!busy && now - lastRequestMs >= 1000){
      sensor.requestTemperatures();
      busy = true;
      lastRequestMs = now;
    }

    if (busy && sensor.isConversionComplete()){
      lastTemp = sensor.getTempCByIndex(0);
      busy = false;
    }
  }

  float temperatureC() const { return lastTemp; }

private:
  DallasTemperature& sensor;
  uint32_t lastRequestMs = 0;
  bool busy = false;
  float lastTemp = NAN;
};

struct TaskInputArgs {
  QueueHandle_t qInput;
};

struct TaskUIArgs {
  QueueHandle_t qInput;
  QueueHandle_t qDisplay;
  QueueHandle_t qCalib;
  EventGroupHandle_t flags;
};

struct TaskSensorArgs {
  QueueHandle_t qDisplay;
  QueueHandle_t qTelemetry;
  QueueHandle_t qCalib;
  SemaphoreHandle_t rs485;
  EventGroupHandle_t flags;
  RuntimeStatus* status;
  AsyncDS18B20* ds;
};

struct TaskHTTPArgs {
  QueueHandle_t qTelemetry;
  EventGroupHandle_t flags;
  RuntimeStatus* status;
};

// Helper RS485
static inline void rs485Receive()  { digitalWrite(RS485_DE_RE_PIN, LOW); }
static inline void rs485Transmit() { digitalWrite(RS485_DE_RE_PIN, HIGH); }

// =======================
//  RTOS PRIMITIVES
// =======================
QueueHandle_t qInput;      // Input events -> UI
QueueHandle_t qDisplay;    // Snapshot sensor -> UI (len=1, overwrite)
QueueHandle_t qTelemetry;  // Snapshot sensor -> HTTP (len=1, overwrite)
QueueHandle_t qCalib;      // Perintah kalibrasi -> TaskSensors
SemaphoreHandle_t mRS485;  // Mutex RS485
EventGroupHandle_t egFlags;

// Event flags
#define EG_WIFI_OK   (1<<0)
#define EG_TIME_OK   (1<<1)
#define EG_PORTAL_ON (1<<2)

// Input event
enum InputType: uint8_t { ENC_DELTA, BTN_PRESS };
struct InputEvent {
  InputType type;
  int16_t   value;   // delta (+1/-1) atau pin id
};

// Kalibrasi command (diekskusi di TaskSensors)
enum class CalibCmd: uint8_t {
  NONE,
  EC_1413, EC_12880,
  NH4_SET_1, NH4_SET_10, NH4_ZERO, NH4_SLOPE, NH4_SET_TEMP25,
  DO_TEMP_FROM_DS, DO_ZERO, DO_SLOPE,
  PH_401, PH_700, PH_1001, PH_TC_EXT, PH_TC_OFF, PH_TC_ONB
};
struct CalibMsg {
  CalibCmd cmd;
};

// Register map (from vendor datasheets)
static const uint16_t REG_EC_CAL_1413   = 0x0030; // write 0xFFFF to auto-cal at 1413 uS/cm
static const uint16_t REG_EC_CAL_12880  = 0x0031; // write 0xFFFF to auto-cal at 12880 uS/cm
static const uint16_t REG_PH_CAL_401    = 0x0030; // write 0xFFFF to auto-cal pH 4.01
static const uint16_t REG_PH_CAL_700    = 0x0031; // write 0xFFFF to auto-cal pH 7.00
static const uint16_t REG_PH_CAL_1001   = 0x0032; // write 0xFFFF to auto-cal pH 10.01
static const uint16_t REG_PH_TC_MODE    = 0x0020; // 0 external, 1 disabled, 2 onboard
static const uint16_t REG_DO_TEMP_CAL   = 0x1000; // value = temp*10
static const uint16_t REG_DO_ZERO       = 0x1001; // write 0
static const uint16_t REG_DO_SLOPE      = 0x1003; // write 0

// =======================
//  GLOBAL STATUS
// =======================
RuntimeStatus gStatus;
AsyncDS18B20 dsAsync(ds18b20);

// Battery sensor (SEN-0052)
#define BAT_ADC_PIN 1
static const float BAT_V_FULL   = 16.8f;
static const float BAT_V_EMPTY  = 12.0f;
static const float BAT_DIV_RATIO = 5.0f; // SEN-0052 typical 1/5 divider
static const uint8_t BAT_SAMPLES = 8;

TaskInputArgs   gInputArgs{};
TaskUIArgs      gUIArgs{};
TaskSensorArgs  gSensorArgs{};
TaskHTTPArgs    gHTTPArgs{};

// Menu cursors
int menuCursor=0, calCursor=0, ecCalCursor=0, nh4CalCursor=0, doCalCursor=0, phCalCursor=0;

// Toast (non-blocking)
static uint32_t toastUntil = 0;
static char toastMsg[12] = {0};

// =======================
//  WAKTU & NTP
// =======================
String makeTimestamp() {
  time_t now = time(nullptr);
  struct tm info;
  localtime_r(&now, &info);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S%z", &info);
  return String(buf);
}

void setupTime(EventGroupHandle_t flags) {
  // Bisa kamu naikkan ke atas file jadi beberapa NTP server:
  // const char* NTP_SERVER   = "pool.ntp.org";
  // const char* NTP_SERVER_2 = "time.google.com";
  // const char* NTP_SERVER_3 = "id.pool.ntp.org";

  configTime(GMT_OFFSET, DAYLIGHT_OFF,
             NTP_SERVER, /* fallback */ "time.google.com", "id.pool.ntp.org");

  struct tm timeinfo;
  bool ok = false;

  // Coba sampai ~10 detik (20 × 500 ms)
  for (int i = 0; i < 20; ++i) {
    if (getLocalTime(&timeinfo, 500)) {
      ok = true;
      break;
    }
  }

  if (ok) {
    xEventGroupSetBits(flags, EG_TIME_OK);
    Serial.println("[NTP] Sync OK");
  } else {
    xEventGroupClearBits(flags, EG_TIME_OK);
    Serial.println("[NTP] Sync FAILED");
  }
}

// void setupTime(EventGroupHandle_t flags) {
//   // Pakai beberapa server NTP sekaligus (fallback)
//   configTime(GMT_OFFSET, DAYLIGHT_OFF,
//              NTP_SERVER, NTP_SERVER_2, NTP_SERVER_3);

//   struct tm timeinfo;
//   bool ok = false;

//   // Coba sampai ~10 detik (20 x 500 ms)
//   for (int i = 0; i < 20; ++i) {
//     // versi ESP32: getLocalTime(&tm, timeout_ms)
//     if (getLocalTime(&timeinfo, 500)) {
//       ok = true;
//       break;
//     }
//   }

//   if (ok) {
//     xEventGroupSetBits(flags, EG_TIME_OK);   // NTP sudah sync
//   } else {
//     xEventGroupClearBits(flags, EG_TIME_OK); // gagal sync
//   }
// }

String formatLocalTime() {
  time_t now = time(nullptr);
  struct tm info;
  localtime_r(&now, &info);
  char buf[24];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &info);
  return String(buf);
}

// Toast helper
void showToast(const char* msg, uint16_t ms=1200){
  strncpy(toastMsg, msg, sizeof(toastMsg)-1);
  toastMsg[sizeof(toastMsg)-1] = '\0';
  toastUntil = millis() + ms;
}
bool toastActive(){ return toastMsg[0] && millis() < toastUntil; }
void clearToast(){ toastMsg[0] = '\0'; }

// =======================
//  HTTP POST
// =======================
bool postData(const DisplayData& s, RuntimeStatus& status) {
  if (WiFi.status() != WL_CONNECTED) {
    strncpy(status.postStatus, "WiFi--", sizeof(status.postStatus)-1);
    status.postStatus[sizeof(status.postStatus)-1] = '\0';
    return false;
  }

  HTTPClient http;
  http.begin(POST_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("accept", "application/json");
  http.setTimeout(HTTP_TIMEOUT_MS);

  String ts = makeTimestamp();

  String payload = "{";
  payload += "\"uid\":\""      + String(UID)           + "\",";
  payload += "\"suhu\":"       + String(s.t_ds,   2)   + ",";
  payload += "\"ph\":"         + String(s.ph,     2)   + ",";
  payload += "\"do\":"         + String(s.do_mgL, 2)   + ",";
  payload += "\"tds\":"        + String(s.tds,    2)   + ",";
  payload += "\"ammonia\":"    + String(s.nh4,    3)   + ",";
  payload += "\"salinitas\":"  + String(s.sal,    2)   + ",";
  if (s.bat_ok){
    payload += "\"battery_v\":"   + String(s.bat_v,   2) + ",";
    payload += "\"battery_pct\":" + String(s.bat_pct, 1) + ",";
  } else {
    payload += "\"battery_v\":null,";
    payload += "\"battery_pct\":null,";
  }
  payload += "\"timestamp\":\""+ ts                   + "\"";
  payload += "}";

  int code = http.POST(payload);
  if (code > 0) {
    if (code == 200 || code == 201) {
      strncpy(status.postStatus, "OK", sizeof(status.postStatus)-1);
      status.postStatus[sizeof(status.postStatus)-1] = '\0';
      http.end();
      return true;
    } else {
      String msg = "E" + String(code);
      strncpy(status.postStatus, msg.c_str(), sizeof(status.postStatus)-1);
      status.postStatus[sizeof(status.postStatus)-1] = '\0';
    }
  } else {
    strncpy(status.postStatus, "HTTPERR", sizeof(status.postStatus)-1);
    status.postStatus[sizeof(status.postStatus)-1] = '\0';
  }

  http.end();
  return false;
}

// =======================
//  PEMBACAAN SENSOR
// =======================
static bool readBattery(float& vbat, float& pct){
  uint32_t acc = 0;
  for (uint8_t i = 0; i < BAT_SAMPLES; ++i){
    acc += analogReadMilliVolts(BAT_ADC_PIN);
  }
  float v_adc = (acc / static_cast<float>(BAT_SAMPLES)) / 1000.0f;
  vbat = v_adc * BAT_DIV_RATIO;
  if (vbat < 0.1f || isnan(vbat)){
    pct = 0.0f;
    return false;
  }
  pct = (vbat - BAT_V_EMPTY) * 100.0f / (BAT_V_FULL - BAT_V_EMPTY);
  pct = constrain(pct, 0.0f, 100.0f);
  return true;
}

float regsToFloatBE(uint16_t r0, uint16_t r1){
  uint8_t b[4] = { uint8_t(r0>>8), uint8_t(r0&0xFF), uint8_t(r1>>8), uint8_t(r1&0xFF) };
  float f;
  memcpy(&f, b, sizeof(f));
  return f;
}

bool readPH(float& t, float& ph){
  uint8_t res = mb_ph.readInputRegisters(0x0001, 2);
  if (res == mb_ph.ku8MBSuccess){
    uint16_t rawPh = mb_ph.getResponseBuffer(0);
    uint16_t rawT  = mb_ph.getResponseBuffer(1);
    ph = rawPh / 100.0f;
    t  = rawT  / 100.0f;
    return true;
  }
  return false;
}

bool readEC(float& t, float& ec, float& tds, float& sal){
  uint8_t res = mb_ec.readInputRegisters(0x0001, 4);
  if (res == mb_ec.ku8MBSuccess){
    uint16_t rawT   = mb_ec.getResponseBuffer(0);
    uint16_t rawEC  = mb_ec.getResponseBuffer(1);
    uint16_t rawTDS = mb_ec.getResponseBuffer(2);
    uint16_t rawSal = mb_ec.getResponseBuffer(3);
    t   = rawT;
    ec  = rawEC;
    tds = rawTDS;
    sal = rawSal;
    return true;
  }
  return false;
}

// Helper pow10u untuk NH4 dan DO
static float pow10u(uint16_t n) {
  float s = 1.0f;
  for (uint16_t i = 0; i < n && i < 6; ++i) s *= 10.0f;
  return s;
}

bool readNH4(float& nh4, float& t){
  uint8_t res = mb_nh4.readHoldingRegisters(0x0000, 4);
  if (res == mb_nh4.ku8MBSuccess){
    uint16_t rawVal    = mb_nh4.getResponseBuffer(0);
    uint16_t rawDecVal = mb_nh4.getResponseBuffer(1);
    uint16_t rawTemp   = mb_nh4.getResponseBuffer(2);
    uint16_t rawDecT   = mb_nh4.getResponseBuffer(3);

    float scaleVal = pow10u(rawDecVal);
    float scaleT   = pow10u(rawDecT);

    nh4 = rawVal  / (scaleVal > 0 ? scaleVal : 1.0f);
    t   = rawTemp / (scaleT   > 0 ? scaleT   : 1.0f);

#if DEBUG_MODBUS
    Serial.print("[NH4] rawVal=");    Serial.print(rawVal);
    Serial.print(" decVal=");         Serial.print(rawDecVal);
    Serial.print(" rawTemp=");        Serial.print(rawTemp);
    Serial.print(" decTemp=");        Serial.println(rawDecT);
    Serial.print("[NH4] NH4=");       Serial.print(nh4);
    Serial.print(" mg/L, T=");        Serial.println(t);
#endif
    return true;
  } else {
#if DEBUG_MODBUS
    Serial.print("[NH4] Modbus error: 0x");
    Serial.println(res, HEX);
#endif
    return false;
  }
}

bool readDO(float& do_mg, float& tC){
  uint8_t res = mb_do.readHoldingRegisters(0x0000, 4);
  if (res == mb_do.ku8MBSuccess){
    uint16_t rawVal    = mb_do.getResponseBuffer(0);
    uint16_t rawDecVal = mb_do.getResponseBuffer(1);
    uint16_t rawTemp   = mb_do.getResponseBuffer(2);
    uint16_t rawDecT   = mb_do.getResponseBuffer(3);

    float scaleVal = pow10u(rawDecVal);
    float scaleT   = pow10u(rawDecT);

    do_mg = rawVal  / (scaleVal > 0 ? scaleVal : 1.0f);
    tC    = rawTemp / (scaleT   > 0 ? scaleT   : 1.0f);

#if DEBUG_MODBUS
    Serial.print("[DO] rawVal=");    Serial.print(rawVal);
    Serial.print(" decVal=");        Serial.print(rawDecVal);
    Serial.print(" rawTemp=");       Serial.print(rawTemp);
    Serial.print(" decTemp=");       Serial.println(rawDecT);
    Serial.print("[DO] DO=");        Serial.print(do_mg);
    Serial.print(" mg/L, T=");       Serial.println(tC);
#endif
    return true;
  } else {
#if DEBUG_MODBUS
    Serial.print("[DO] Modbus error: 0x");
    Serial.println(res, HEX);
#endif
    return false;
  }
}

// =======================
//  PEMBACAAN RK500-09 MULTI SENSOR
// =======================
//
// Mapping index register dari manual RK500-09:
// idx 0:  Temp value,  idx 1: Temp dec
// idx 6:  EC value,    idx 7: EC dec
// idx 8:  pH value,    idx 9: pH dec
// idx 12: DO value,    idx 13: DO dec
// idx 14: NH4 value,   idx 15: NH4 dec
// idx 34: Sal value,   idx 35: Sal dec
// idx 36: TDS value,   idx 37: TDS dec
//
// Nilai akhir = value / (10^dec)
//

static bool readRK50009(DisplayData& cur) {
  // Baca 0x002A (42) register mulai address 0
  uint8_t res = mb_rk.readHoldingRegisters(0x0000, 0x002A);
  if (res != mb_rk.ku8MBSuccess) {
    // kalau gagal, tandai semua parameter dari RK500-09 sebagai invalid
    cur.ph_ok  = false;
    cur.ec_ok  = false;
    cur.do_ok  = false;
    cur.nh4_ok = false;
    return false;
  }

  auto val_dec = [&](uint16_t idxVal, uint16_t idxDec) -> float {
    uint16_t rawVal = mb_rk.getResponseBuffer(idxVal);
    uint16_t rawDec = mb_rk.getResponseBuffer(idxDec);
    float scale = pow10u(rawDec);
    return rawVal / (scale > 0.0f ? scale : 1.0f);
  };

  // === SUHU INTERNAL RK500-09 ===
  float t_rk  = val_dec(0, 1);   // <-- suhu dari RK500-09 (INI YANG AKAN KITA PAKAI)

  // Parameter utama
  float ec    = val_dec(6, 7);    // uS/cm
  float ph    = val_dec(8, 9);    // pH
  float do_mg = val_dec(12, 13);  // mg/L
  float nh4   = val_dec(14, 15);  // mg/L
  float sal   = val_dec(34, 35);  // PSU
  float tds   = val_dec(36, 37);  // mg/L

  // ===== MASUKKAN KE DisplayData =====

  // 1) SUHU UTAMA ALAT
  //    t_ds sekarang DIISI dari RK500-09, bukan dari DS18B20
  cur.t_ds   = t_rk;

  // 2) Parameter lain
  cur.ec     = ec;
  cur.ph     = ph;
  cur.do_mgL = do_mg;
  cur.nh4    = nh4;
  cur.sal    = sal;
  cur.tds    = tds;

  // 3) Simpan juga sebagai suhu internal sensor-sensor (kalau mau dipakai di kalibrasi)
  cur.phT    = t_rk;
  cur.ecT    = t_rk;
  cur.do_tC  = t_rk;
  cur.nh4T   = t_rk;

  cur.ph_ok  = true;
  cur.ec_ok  = true;
  cur.do_ok  = true;
  cur.nh4_ok = true;

  return true;
}


// =======================
//  MODBUS WRITE HELPER (Calib)
// =======================
static bool writeReg(ModbusMaster& mb, uint16_t reg, uint16_t value){
  uint8_t res = mb.writeSingleRegister(reg, value);
  return res == mb.ku8MBSuccess;
}

static void handleCalibration(CalibCmd cmd, DisplayData& cur){
  bool ok = false;
  switch (cmd){
    case CalibCmd::EC_1413:
      ok = writeReg(mb_ec, REG_EC_CAL_1413, 0xFFFF);
      break;
    case CalibCmd::EC_12880:
      ok = writeReg(mb_ec, REG_EC_CAL_12880, 0xFFFF);
      break;
    case CalibCmd::DO_TEMP_FROM_DS:
      if (!isnan(cur.t_ds)){
        ok = writeReg(mb_do, REG_DO_TEMP_CAL, static_cast<uint16_t>(cur.t_ds * 10.0f));
      }
      break;
    case CalibCmd::DO_ZERO:
      ok = writeReg(mb_do, REG_DO_ZERO, 0);
      break;
    case CalibCmd::DO_SLOPE:
      ok = writeReg(mb_do, REG_DO_SLOPE, 0);
      break;
    case CalibCmd::PH_401:
      ok = writeReg(mb_ph, REG_PH_CAL_401, 0xFFFF);
      break;
    case CalibCmd::PH_700:
      ok = writeReg(mb_ph, REG_PH_CAL_700, 0xFFFF);
      break;
    case CalibCmd::PH_1001:
      ok = writeReg(mb_ph, REG_PH_CAL_1001, 0xFFFF);
      break;
    case CalibCmd::PH_TC_EXT:
      ok = writeReg(mb_ph, REG_PH_TC_MODE, 0); // external probe
      break;
    case CalibCmd::PH_TC_OFF:
      ok = writeReg(mb_ph, REG_PH_TC_MODE, 1); // disabled
      break;
    case CalibCmd::PH_TC_ONB:
      ok = writeReg(mb_ph, REG_PH_TC_MODE, 2); // onboard
      break;
    default:
      break;
  }

  if (ok) {
    showToast("Cal OK");
  } else if (cmd != CalibCmd::NONE){
    showToast("Cal Fail");
  }
}

// =======================
//  INPUT / DEBOUNCE
// =======================
bool debounceRead(uint8_t pin){
  static uint8_t last[64] = {1};
  static uint32_t tLast[64] = {0};
  uint8_t raw = digitalRead(pin);
  uint32_t now = millis();
  if (raw != last[pin] && now - tLast[pin] > 25){
    tLast[pin] = now; last[pin] = raw;
    if (raw == LOW) return true;
  }
  return false;
}

// =======================
//  MENU DATA
// =======================
const char* MAIN_ITEMS[] = {
  "Dashboard",
  "WiFi Manager",
  "Kalibrasi Sensor"
};
const int MAIN_COUNT = sizeof(MAIN_ITEMS)/sizeof(MAIN_ITEMS[0]);

const char* CAL_ITEMS[] = {
  "Cal EC (Auto)",
  "Cal NH4",
  "Cal DO",
  "Cal pH (S-PH-01)",
  "Back"
};
const int CAL_COUNT = sizeof(CAL_ITEMS)/sizeof(CAL_ITEMS[0]);

const char* EC_CAL_ITEMS[] = {
  "EC 1413",
  "EC 12880",
  "Back"
};
const int EC_CAL_COUNT = sizeof(EC_CAL_ITEMS)/sizeof(EC_CAL_ITEMS[0]);

const char* NH4_CAL_ITEMS[] = {
  "Set 1 mg/L",
  "Set 10 mg/L",
  "Zero",
  "Slope",
  "Set Temp=25C",
  "Back"
};
const int NH4_CAL_COUNT = sizeof(NH4_CAL_ITEMS)/sizeof(NH4_CAL_ITEMS[0]);

const char* DO_CAL_ITEMS[] = {
  "Temp from DS18B20",
  "Zero",
  "Slope",
  "Back"
};
const int DO_CAL_COUNT = sizeof(DO_CAL_ITEMS)/sizeof(DO_CAL_ITEMS[0]);

const char* PH_CAL_ITEMS[] = {
  "pH 4.01",
  "pH 7.00",
  "pH 10.01",
  "TC External",
  "TC Off",
  "TC On Board",
  "Back"
};
const int PH_CAL_COUNT = sizeof(PH_CAL_ITEMS)/sizeof(PH_CAL_ITEMS[0]);

const char* WIFI_MGR_ITEMS[] = { "Start Portal", "Back" };
const int WIFI_MGR_COUNT = sizeof(WIFI_MGR_ITEMS)/sizeof(WIFI_MGR_ITEMS[0]);

// =======================
//  DRAW HELPERS (UI)
// =======================
struct DashboardCache {
  char temp[16];
  char do_mg[16];
  char ph[16];
  char nh4[16];
  char ec[16];
  char tds[16];
  char statusLeft[48];
  char statusRight[24];
  bool valid = false;
};

static void formatDashboardStrings(const DisplayData& d, DashboardCache& out){
  snprintf(out.temp, sizeof(out.temp), "%.2f C", d.t_ds);
  if (d.do_ok)  snprintf(out.do_mg, sizeof(out.do_mg), "%.2f mg/L", d.do_mgL);
  else          snprintf(out.do_mg, sizeof(out.do_mg), "--");
  if (d.ph_ok)  snprintf(out.ph, sizeof(out.ph), "%.2f", d.ph);
  else          snprintf(out.ph, sizeof(out.ph), "--");
  if (d.nh4_ok) snprintf(out.nh4, sizeof(out.nh4), "%.2f mg/L", d.nh4);
  else          snprintf(out.nh4, sizeof(out.nh4), "--");
  if (d.ec_ok)  snprintf(out.ec, sizeof(out.ec), "%.0f uS/cm", d.ec);
  else          snprintf(out.ec, sizeof(out.ec), "--");
  if (d.ec_ok)  snprintf(out.tds, sizeof(out.tds), "%.0f mg/L", d.tds);
  else          snprintf(out.tds, sizeof(out.tds), "--");

  snprintf(out.statusLeft, sizeof(out.statusLeft), "WiFi:%s NTP:%s POST:%s",
           d.wifiOK ? "OK" : "--",
           d.ntpOK  ? "OK" : "--",
           d.postStatus);
  if (d.bat_ok){
    snprintf(out.statusRight, sizeof(out.statusRight), "BAT %d%% %.1fV",
             static_cast<int>(d.bat_pct + 0.5f), d.bat_v);
  } else {
    snprintf(out.statusRight, sizeof(out.statusRight), "BAT --");
  }
}

static void drawToastOverlay(){
  if (!toastActive()){
    return;
  }
  tft.fillRect(0, 24, 320, 18, TFT_ORANGE);
  tft.setTextColor(TFT_BLACK, TFT_ORANGE);
  tft.setTextFont(2);
  tft.drawString(toastMsg, 6, 26);
}

static void drawStatusBar(const DisplayData& d){
  tft.fillRect(0, 0, 320, 24, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextFont(2);

  DashboardCache tmp{};
  formatDashboardStrings(d, tmp);
  tft.drawString(tmp.statusLeft, 6, 4);

  int16_t w = tft.textWidth(tmp.statusRight, 2);
  tft.drawString(tmp.statusRight, 316 - w, 4);
}

static void drawMetricBox(int x, int y, int w, int h, const char* label, const char* value, uint16_t color){
  tft.drawRect(x, y, w, h, TFT_DARKGREY);
  tft.setTextFont(2);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString(label, x + 6, y + 4);
  tft.setTextFont(4);
  tft.setTextColor(color, TFT_BLACK);
  tft.drawString(value, x + 6, y + 24);
}

void drawSplashFrame(uint8_t pct, bool wifiOK, bool ntpOK){
  const uint16_t splashBg = tft.color565(0x19, 0x8A, 0xFF);
  tft.fillScreen(splashBg);

  tft.setTextFont(4);
  tft.setTextColor(TFT_WHITE, splashBg);
  tft.drawCentreString("Aerasea", 160, 10, 4);

  int logoX = (320 - AERASEA_LOGO_WIDTH) / 2;
  int logoY = 40;
  tft.drawXBitmap(logoX, logoY, aerasea_logo_128x64_inverted_bits,
                  AERASEA_LOGO_WIDTH, AERASEA_LOGO_HEIGHT, TFT_WHITE);

  tft.setTextFont(2);
  tft.setTextColor(TFT_WHITE, splashBg);
  char ln1[28]; snprintf(ln1, sizeof(ln1), "UID: %s", UID);
  char ln2[20]; snprintf(ln2, sizeof(ln2), "FW : %s", FW_VERSION);
  tft.drawCentreString(ln1, 160, 115, 2);
  tft.drawCentreString(ln2, 160, 135, 2);

  tft.setTextColor(TFT_WHITE, splashBg);
  tft.drawCentreString(wifiOK ? "WiFi: OK" : "WiFi: --", 110, 158, 2);
  tft.drawCentreString(ntpOK  ? "NTP: OK"  : "NTP: --", 210, 158, 2);

  tft.drawRect(40, 190, 240, 14, TFT_WHITE);
  tft.fillRect(42, 192, map(pct, 0, 100, 0, 236), 10, TFT_WHITE);
}

static void drawDashboardFrame(const DisplayData& d){
  tft.fillScreen(TFT_BLACK);
  drawStatusBar(d);

  const int gap = 6;
  const int colW = (320 - gap * 3) / 2;
  const int rowH = 60;
  const int y0 = 28;

  char buf[24];

  snprintf(buf, sizeof(buf), "%.2f C", d.t_ds);
  drawMetricBox(gap, y0, colW, rowH, "Temperature", buf, TFT_YELLOW);

  if (d.do_ok) snprintf(buf, sizeof(buf), "%.2f mg/L", d.do_mgL);
  else         snprintf(buf, sizeof(buf), "--");
  drawMetricBox(gap * 2 + colW, y0, colW, rowH, "Dissolved O2", buf, TFT_CYAN);

  if (d.ph_ok) snprintf(buf, sizeof(buf), "%.2f", d.ph);
  else         snprintf(buf, sizeof(buf), "--");
  drawMetricBox(gap, y0 + rowH + gap, colW, rowH, "pH", buf, TFT_GREEN);

  if (d.nh4_ok) snprintf(buf, sizeof(buf), "%.2f mg/L", d.nh4);
  else          snprintf(buf, sizeof(buf), "--");
  drawMetricBox(gap * 2 + colW, y0 + rowH + gap, colW, rowH, "NH4", buf, TFT_ORANGE);

  if (d.ec_ok) snprintf(buf, sizeof(buf), "%.0f uS/cm", d.ec);
  else         snprintf(buf, sizeof(buf), "--");
  drawMetricBox(gap, y0 + (rowH + gap) * 2, colW, rowH, "EC", buf, TFT_BLUE);

  if (d.ec_ok) snprintf(buf, sizeof(buf), "%.0f mg/L", d.tds);
  else         snprintf(buf, sizeof(buf), "--");
  drawMetricBox(gap * 2 + colW, y0 + (rowH + gap) * 2, colW, rowH, "TDS", buf, TFT_MAGENTA);

  drawToastOverlay();
}

static void drawDashboardUpdate(const DisplayData& d, DashboardCache& cache, bool force){
  DashboardCache now{};
  formatDashboardStrings(d, now);

  if (force || !cache.valid || strncmp(cache.statusLeft, now.statusLeft, sizeof(now.statusLeft)) != 0 ||
      strncmp(cache.statusRight, now.statusRight, sizeof(now.statusRight)) != 0){
    drawStatusBar(d);
  }

  const int gap = 6;
  const int colW = (320 - gap * 3) / 2;
  const int rowH = 60;
  const int y0 = 28;

  if (force || !cache.valid || strncmp(cache.temp, now.temp, sizeof(now.temp)) != 0){
    drawMetricBox(gap, y0, colW, rowH, "Temperature", now.temp, TFT_YELLOW);
  }
  if (force || !cache.valid || strncmp(cache.do_mg, now.do_mg, sizeof(now.do_mg)) != 0){
    drawMetricBox(gap * 2 + colW, y0, colW, rowH, "Dissolved O2", now.do_mg, TFT_CYAN);
  }
  if (force || !cache.valid || strncmp(cache.ph, now.ph, sizeof(now.ph)) != 0){
    drawMetricBox(gap, y0 + rowH + gap, colW, rowH, "pH", now.ph, TFT_GREEN);
  }
  if (force || !cache.valid || strncmp(cache.nh4, now.nh4, sizeof(now.nh4)) != 0){
    drawMetricBox(gap * 2 + colW, y0 + rowH + gap, colW, rowH, "NH4", now.nh4, TFT_ORANGE);
  }
  if (force || !cache.valid || strncmp(cache.ec, now.ec, sizeof(now.ec)) != 0){
    drawMetricBox(gap, y0 + (rowH + gap) * 2, colW, rowH, "EC", now.ec, TFT_BLUE);
  }
  if (force || !cache.valid || strncmp(cache.tds, now.tds, sizeof(now.tds)) != 0){
    drawMetricBox(gap * 2 + colW, y0 + (rowH + gap) * 2, colW, rowH, "TDS", now.tds, TFT_MAGENTA);
  }

  cache = now;
  cache.valid = true;
  drawToastOverlay();
}

void drawMenu(const char* title, const char* items[], int nItems, int cursor){
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 320, 24, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextFont(2);
  tft.drawString(title, 6, 4);

  int y = 34;
  const int itemH = 28;
  for (int i = 0; i < nItems; ++i){
    if (i == cursor){
      tft.fillRect(6, y - 4, 308, itemH, TFT_BLUE);
      tft.setTextColor(TFT_WHITE, TFT_BLUE);
    } else {
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }
    tft.setTextFont(2);
    tft.drawString(items[i], 12, y);
    y += itemH;
  }

  drawToastOverlay();
}

static void drawMenuItem(const char* items[], int index, bool selected){
  const int itemH = 28;
  const int y = 34 + index * itemH;
  tft.fillRect(6, y - 4, 308, itemH, selected ? TFT_BLUE : TFT_BLACK);
  tft.setTextColor(TFT_WHITE, selected ? TFT_BLUE : TFT_BLACK);
  tft.setTextFont(2);
  tft.drawString(items[index], 12, y);
}

// =======================
//  LED INDICATOR (RGB)
// =======================
struct LedRGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

static inline void ledWrite(const LedRGB& c){
  ledcWrite(LEDC_CH_R, c.r);
  ledcWrite(LEDC_CH_G, c.g);
  ledcWrite(LEDC_CH_B, c.b);
}

static void ledInit(){
  ledcSetup(LEDC_CH_R, LEDC_FREQ, LEDC_RES);
  ledcSetup(LEDC_CH_G, LEDC_FREQ, LEDC_RES);
  ledcSetup(LEDC_CH_B, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(LED_R_PIN, LEDC_CH_R);
  ledcAttachPin(LED_G_PIN, LEDC_CH_G);
  ledcAttachPin(LED_B_PIN, LEDC_CH_B);
  ledWrite({0,0,0});
}

// =======================
//  TASK INPUT (4 BUTTON)
// =======================
void TaskInput(void* param){
  auto* args = static_cast<TaskInputArgs*>(param);
  if (!args || !args->qInput){
    vTaskDelete(NULL);
    return;
  }

  pinMode(BTN_UP,    INPUT_PULLUP);
  pinMode(BTN_DOWN,  INPUT_PULLUP);
  pinMode(BTN_OK,    INPUT_PULLUP);
  pinMode(BTN_BACK,  INPUT_PULLUP);

  for(;;){
    // Navigasi: UP / DOWN → ENC_DELTA (-1 / +1)
    if (debounceRead(BTN_UP)){
      InputEvent e{ENC_DELTA, -1};
      xQueueSend(args->qInput, &e, 0);
    }
    if (debounceRead(BTN_DOWN)){
      InputEvent e{ENC_DELTA, +1};
      xQueueSend(args->qInput, &e, 0);
    }

    // Aksi: OK & BACK → BTN_PRESS
    if (debounceRead(BTN_OK)){
      InputEvent e{BTN_PRESS, BTN_OK};
      xQueueSend(args->qInput, &e, 0);
    }
    if (debounceRead(BTN_BACK)){
      InputEvent e{BTN_PRESS, BTN_BACK};
      xQueueSend(args->qInput, &e, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// =======================
//  TASK UI
// =======================
void TaskUI(void* param){
  auto* args = static_cast<TaskUIArgs*>(param);
  if (!args || !args->qInput || !args->qDisplay || !args->flags || !args->qCalib){
    vTaskDelete(NULL);
    return;
  }

  DisplayData disp = {};
  DashboardCache dashCache{};
  uint8_t splashPct = 0;
  uint32_t tSplash = millis();
  UIState ui = UIState::DASHBOARD;
  UIState lastUi = UIState::DASHBOARD;
  bool lastWifiOK = false;
  bool lastNtpOK = false;
  bool lastToastActive = false;
  char lastToastMsg[sizeof(toastMsg)] = {0};
  char lastPostStatus[16] = {0};
  int lastMenuCursor = menuCursor;
  int lastCalCursor = calCursor;
  int lastEcCalCursor = ecCalCursor;
  int lastNh4CalCursor = nh4CalCursor;
  int lastDoCalCursor = doCalCursor;
  int lastPhCalCursor = phCalCursor;
  uint8_t modbusFailStreak = 0;
  uint8_t postErrStreak = 0;
  uint32_t ledFlashUntil = 0;
  LedRGB ledFlashColor{0,0,0};

  for(;;){
    bool gotInput = false;
    bool gotDisplay = false;
    bool forceRedraw = false;

    InputEvent ev;
    while (xQueueReceive(args->qInput, &ev, 0) == pdTRUE){
      gotInput = true;
      if (ev.type == ENC_DELTA){
        int dir = (ev.value > 0) ? 1 : -1;
        switch (ui){
          case UIState::MENU:
            menuCursor = constrain(menuCursor + dir, 0, MAIN_COUNT-1);
            break;
          case UIState::CALIB:
            calCursor = constrain(calCursor + dir, 0, CAL_COUNT-1);
            break;
          case UIState::CAL_EC:
            ecCalCursor = constrain(ecCalCursor + dir, 0, EC_CAL_COUNT-1);
            break;
          case UIState::CAL_NH4:
            nh4CalCursor = constrain(nh4CalCursor + dir, 0, NH4_CAL_COUNT-1);
            break;
          case UIState::CAL_DO:
            doCalCursor = constrain(doCalCursor + dir, 0, DO_CAL_COUNT-1);
            break;
          case UIState::CAL_PH:
            phCalCursor = constrain(phCalCursor + dir, 0, PH_CAL_COUNT-1);
            break;
          default:
            break;
        }
      } else if (ev.type == BTN_PRESS){
        if (ui==UIState::MENU || ui==UIState::WIFI_MGR || ui==UIState::CALIB ||
            ui==UIState::CAL_EC || ui==UIState::CAL_NH4 || ui==UIState::CAL_DO || ui==UIState::CAL_PH){
          if (ev.value==BTN_BACK){
            ui = UIState::DASHBOARD;
            showToast("Back");
            continue;
          }
        }

        if (ui == UIState::DASHBOARD){
          if (ev.value == BTN_OK){
            ui = UIState::MENU;
            showToast("Menu");
          }
        } else if (ui == UIState::MENU){
          if (ev.value == BTN_OK){
            switch (menuCursor){
              case 0: ui = UIState::DASHBOARD;   break;
              case 1: ui = UIState::WIFI_MGR;    break;
              case 2: ui = UIState::CALIB;       break;
            }
          }
        } else if (ui == UIState::WIFI_MGR){
          if (ev.value == BTN_OK){
            xEventGroupSetBits(args->flags, EG_PORTAL_ON);
            showToast("WiFi Portal");
            ui = UIState::DASHBOARD;
          }
        } else if (ui == UIState::CALIB){
          if (ev.value == BTN_OK){
            switch (calCursor){
              case 0: ui = UIState::CAL_EC;  break;
              case 1: ui = UIState::CAL_NH4; break;
              case 2: ui = UIState::CAL_DO;  break;
              case 3: ui = UIState::CAL_PH;  break;
              case 4: ui = UIState::DASHBOARD; break;
            }
          }
        } else if (ui == UIState::CAL_NH4){
          if (ev.value == BTN_OK){
            showToast("NH4 cal N/A");
            ui = UIState::DASHBOARD;
          }
        } else if (ui == UIState::CAL_EC){
          if (ev.value == BTN_OK && args->qCalib){
            CalibMsg m{};
            if (ecCalCursor == 0) { m.cmd = CalibCmd::EC_1413; }
            else if (ecCalCursor == 1) { m.cmd = CalibCmd::EC_12880; }
            if (m.cmd != CalibCmd::NONE){
              xQueueSend(args->qCalib, &m, 0);
              showToast("Cal EC");
            }
            ui = UIState::DASHBOARD;
          }
        } else if (ui == UIState::CAL_DO){
          if (ev.value == BTN_OK && args->qCalib){
            CalibMsg m{};
            if (doCalCursor == 0) m.cmd = CalibCmd::DO_TEMP_FROM_DS;
            else if (doCalCursor == 1) m.cmd = CalibCmd::DO_ZERO;
            else if (doCalCursor == 2) m.cmd = CalibCmd::DO_SLOPE;
            if (m.cmd != CalibCmd::NONE){
              xQueueSend(args->qCalib, &m, 0);
              showToast("Cal DO");
            }
            ui = UIState::DASHBOARD;
          }
        } else if (ui == UIState::CAL_PH){
          if (ev.value == BTN_OK && args->qCalib){
            CalibMsg m{};
            switch (phCalCursor){
              case 0: m.cmd = CalibCmd::PH_401; break;
              case 1: m.cmd = CalibCmd::PH_700; break;
              case 2: m.cmd = CalibCmd::PH_1001; break;
              case 3: m.cmd = CalibCmd::PH_TC_EXT; break;
              case 4: m.cmd = CalibCmd::PH_TC_OFF; break;
              case 5: m.cmd = CalibCmd::PH_TC_ONB; break;
              default: break;
            }
            if (m.cmd != CalibCmd::NONE){
              xQueueSend(args->qCalib, &m, 0);
              showToast("Cal pH");
            }
            ui = UIState::DASHBOARD;
          }
        }
      }
    }

    DisplayData tmp;
    if (xQueueReceive(args->qDisplay, &tmp, 0) == pdTRUE){
      disp = tmp;
      gotDisplay = true;

      bool sensorsOK = disp.ph_ok && disp.ec_ok && disp.do_ok && disp.nh4_ok;
      if (sensorsOK){
        modbusFailStreak = 0;
      } else if (modbusFailStreak < 255){
        modbusFailStreak++;
      }

      bool postErr = (disp.postStatus[0] == 'E') || (strncmp(disp.postStatus, "HTTPERR", 7) == 0);
      if (postErr){
        if (postErrStreak < 255) postErrStreak++;
      } else {
        postErrStreak = 0;
      }

      if (strncmp(disp.postStatus, "OK", 2) == 0 && strncmp(lastPostStatus, "OK", 2) != 0){
        ledFlashUntil = millis() + 300;
        ledFlashColor = {0, 192, 192}; // cyan flash
      }
      strncpy(lastPostStatus, disp.postStatus, sizeof(lastPostStatus)-1);
      lastPostStatus[sizeof(lastPostStatus)-1] = '\0';
    }

    EventBits_t flags = xEventGroupGetBits(args->flags);
    bool wifiOK = (flags & EG_WIFI_OK);
    bool ntpOK  = (flags & EG_TIME_OK);
    bool toastNow = toastActive();
    bool toastChanged = (toastNow != lastToastActive) ||
                        (strncmp(toastMsg, lastToastMsg, sizeof(toastMsg)) != 0);

    if (toastChanged){
      if (strncmp(toastMsg, "Cal OK", 6) == 0){
        ledFlashUntil = millis() + 350;
        ledFlashColor = {192, 0, 192}; // magenta flash
      } else if (strncmp(toastMsg, "Cal Fail", 8) == 0){
        ledFlashUntil = millis() + 400;
        ledFlashColor = {192, 0, 0}; // red flash
      }
    }

    if (!toastNow && toastChanged){
      forceRedraw = true;
    }

    uint32_t now = millis();
    bool inSplash = (now - tSplash < 2000);
    bool needDraw = gotInput || gotDisplay || toastChanged ||
                    (wifiOK != lastWifiOK) || (ntpOK != lastNtpOK);

    if (inSplash){
      if (splashPct < 100 && now - tSplash > splashPct*10){
        splashPct++;
        needDraw = true;
      }
      if (needDraw){
        drawSplashFrame(splashPct, wifiOK, ntpOK);
      }
    } else {
      if (needDraw){
        if (ui != lastUi){
          forceRedraw = true;
          lastUi = ui;
          dashCache.valid = false;
        }

        switch (ui){
          case UIState::DASHBOARD:
            if (forceRedraw){
              drawDashboardFrame(disp);
              dashCache.valid = false;
            }
            drawDashboardUpdate(disp, dashCache, forceRedraw);
            break;
          case UIState::MENU:
            if (forceRedraw){
              drawMenu("Main Menu", MAIN_ITEMS, MAIN_COUNT, menuCursor);
              lastMenuCursor = menuCursor;
            } else if (menuCursor != lastMenuCursor){
              drawMenuItem(MAIN_ITEMS, lastMenuCursor, false);
              drawMenuItem(MAIN_ITEMS, menuCursor, true);
              lastMenuCursor = menuCursor;
              drawToastOverlay();
            } else if (toastChanged) {
              drawToastOverlay();
            }
            break;
          case UIState::CALIB:
            if (forceRedraw){
              drawMenu("Kalibrasi Sensor", CAL_ITEMS, CAL_COUNT, calCursor);
              lastCalCursor = calCursor;
            } else if (calCursor != lastCalCursor){
              drawMenuItem(CAL_ITEMS, lastCalCursor, false);
              drawMenuItem(CAL_ITEMS, calCursor, true);
              lastCalCursor = calCursor;
              drawToastOverlay();
            } else if (toastChanged) {
              drawToastOverlay();
            }
            break;
          case UIState::CAL_EC:
            if (forceRedraw){
              drawMenu("Cal EC (Auto)", EC_CAL_ITEMS, EC_CAL_COUNT, ecCalCursor);
              lastEcCalCursor = ecCalCursor;
            } else if (ecCalCursor != lastEcCalCursor){
              drawMenuItem(EC_CAL_ITEMS, lastEcCalCursor, false);
              drawMenuItem(EC_CAL_ITEMS, ecCalCursor, true);
              lastEcCalCursor = ecCalCursor;
              drawToastOverlay();
            } else if (toastChanged) {
              drawToastOverlay();
            }
            break;
          case UIState::CAL_NH4:
            if (forceRedraw){
              drawMenu("Cal NH4", NH4_CAL_ITEMS, NH4_CAL_COUNT, nh4CalCursor);
              lastNh4CalCursor = nh4CalCursor;
            } else if (nh4CalCursor != lastNh4CalCursor){
              drawMenuItem(NH4_CAL_ITEMS, lastNh4CalCursor, false);
              drawMenuItem(NH4_CAL_ITEMS, nh4CalCursor, true);
              lastNh4CalCursor = nh4CalCursor;
              drawToastOverlay();
            } else if (toastChanged) {
              drawToastOverlay();
            }
            break;
          case UIState::CAL_DO:
            if (forceRedraw){
              drawMenu("Cal DO", DO_CAL_ITEMS, DO_CAL_COUNT, doCalCursor);
              lastDoCalCursor = doCalCursor;
            } else if (doCalCursor != lastDoCalCursor){
              drawMenuItem(DO_CAL_ITEMS, lastDoCalCursor, false);
              drawMenuItem(DO_CAL_ITEMS, doCalCursor, true);
              lastDoCalCursor = doCalCursor;
              drawToastOverlay();
            } else if (toastChanged) {
              drawToastOverlay();
            }
            break;
          case UIState::CAL_PH:
            if (forceRedraw){
              drawMenu("Cal pH (S-PH-01)", PH_CAL_ITEMS, PH_CAL_COUNT, phCalCursor);
              lastPhCalCursor = phCalCursor;
            } else if (phCalCursor != lastPhCalCursor){
              drawMenuItem(PH_CAL_ITEMS, lastPhCalCursor, false);
              drawMenuItem(PH_CAL_ITEMS, phCalCursor, true);
              lastPhCalCursor = phCalCursor;
              drawToastOverlay();
            } else if (toastChanged) {
              drawToastOverlay();
            }
            break;
          case UIState::WIFI_MGR:
            if (forceRedraw){
              drawMenu("WiFi Manager", WIFI_MGR_ITEMS, WIFI_MGR_COUNT, 0);
            } else if (toastChanged) {
              drawToastOverlay();
            }
            break;
        }
      }
    }

    if (!toastNow){
      clearToast();
    }

    if (needDraw){
      lastWifiOK = wifiOK;
      lastNtpOK = ntpOK;
      lastToastActive = toastNow;
      strncpy(lastToastMsg, toastMsg, sizeof(lastToastMsg)-1);
      lastToastMsg[sizeof(lastToastMsg)-1] = '\0';
    }

    // ===== LED indicator =====
    const uint32_t nowLed = millis();
    if (ledFlashUntil > nowLed){
      ledWrite(ledFlashColor);
    } else {
      const bool criticalError = (modbusFailStreak >= 3) || (postErrStreak >= 2);
      if (criticalError){
        const bool on = ((nowLed / 200) % 2) == 0;
        ledWrite(on ? LedRGB{192,0,0} : LedRGB{0,0,0}); // fast red blink
      } else if (!wifiOK){
        const bool on = ((nowLed / 1000) % 2) == 0;
        ledWrite(on ? LedRGB{0,0,192} : LedRGB{0,0,0}); // slow blue blink
      } else if (wifiOK && !ntpOK){
        const bool on = ((nowLed / 1000) % 2) == 0;
        ledWrite(on ? LedRGB{192,192,0} : LedRGB{0,0,0}); // slow yellow blink
      } else {
        ledWrite({0,192,0}); // solid green
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =======================
// TaskSensors (Core 0)
// =======================
void TaskSensors(void* param){
  auto* args = static_cast<TaskSensorArgs*>(param);
  if (!args || !args->qDisplay || !args->qTelemetry || !args->qCalib || !args->rs485 || !args->status || !args->ds || !args->flags){
    vTaskDelete(NULL);
    return;
  }

  AsyncDS18B20& ds = *(args->ds);
  ds.begin();

  pinMode(RS485_DE_RE_PIN, OUTPUT); rs485Receive();
  RS485.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  auto bindNode = [](ModbusMaster& mb, uint8_t id){
    mb.begin(id, RS485);
    mb.preTransmission(rs485Transmit);
    mb.postTransmission(rs485Receive);
  };

  // Multi-parameter RK500-09 (address 0x06)
  bindNode(mb_rk, RK_SLAVE_ID);

  // (opsional) legacy node untuk menu kalibrasi lama – tidak dibaca lagi periodik
  bindNode(mb_ph,  PH_SLAVE_ID);
  bindNode(mb_ec,  EC_SLAVE_ID);
  bindNode(mb_nh4, NH4_SLAVE_ID);
  bindNode(mb_do,  DO_SLAVE_ID);

  DisplayData cur = {};
  const TickType_t rsLockTimeout = pdMS_TO_TICKS(100);

  for(;;){
    // 1) Proses perintah kalibrasi (kalau kamu masih pakai menu lama)
    CalibMsg msg;
    while (xQueueReceive(args->qCalib, &msg, 0) == pdTRUE){
      if (xSemaphoreTake(args->rs485, rsLockTimeout) == pdTRUE){
        handleCalibration(msg.cmd, cur);
        xSemaphoreGive(args->rs485);
      }
    }

    // 2) Update suhu DS18B20 (non-blocking)
    // ds.tick();
    // float t = ds.temperatureC();
    // if (!isnan(t)){
    //   cur.t_ds = t;
    // }

    // 3) Baca semua parameter dari RK500-09 sekali jalan
    if (xSemaphoreTake(args->rs485, rsLockTimeout) == pdTRUE){
      readRK50009(cur);
      rs485Receive();
      xSemaphoreGive(args->rs485);
    }

    float vbat = 0.0f;
    float batPct = 0.0f;
    cur.bat_ok = readBattery(vbat, batPct);
    if (cur.bat_ok){
      cur.bat_v = vbat;
      cur.bat_pct = batPct;
    } else {
      cur.bat_v = 0.0f;
      cur.bat_pct = 0.0f;
    }

    // 4) Tambahan: status WiFi, waktu, POST
    cur.wifiOK = (WiFi.status()==WL_CONNECTED);
    EventBits_t flags = xEventGroupGetBits(args->flags);
    cur.ntpOK  = (flags & EG_TIME_OK);
    strncpy(cur.postStatus, args->status->postStatus, sizeof(cur.postStatus)-1);
    cur.postStatus[sizeof(cur.postStatus)-1] = '\0';
    // 5) Kirim snapshot ke UI & HTTP
    xQueueOverwrite(args->qDisplay, &cur);
    xQueueOverwrite(args->qTelemetry, &cur);

    // 6) Delay sampling
    vTaskDelay(pdMS_TO_TICKS(250));   // 4 Hz; silakan sesuaikan
  }
}

// =======================
// TaskHTTP (Core 0, low)
// =======================

// =======================
// TaskHTTP (Core 0, low)
// =======================
void TaskHTTP(void* param){
  auto* args = static_cast<TaskHTTPArgs*>(param);
  if (!args || !args->qTelemetry || !args->status || !args->flags){
    vTaskDelete(NULL);
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin();   // pakai kredensial terakhir yang tersimpan

  WiFiManager wm;
  wm.setConfigPortalTimeout(300);
  wm.setConfigPortalBlocking(false);

  uint32_t lastPost = 0;
  DisplayData snap{};
  bool wifiWasConnected = false;

  for(;;){
    EventBits_t f = xEventGroupGetBits(args->flags);

    // ====== WiFi Manager Portal (dipanggil dari menu) ======
    if (f & EG_PORTAL_ON){
      xEventGroupClearBits(args->flags, EG_PORTAL_ON);

      WiFi.disconnect(true, true);
      vTaskDelay(pdMS_TO_TICKS(200));

      wm.startConfigPortal("Aerasea-Setup");
      uint32_t portalStart = millis();
      while (wm.getConfigPortalActive()){
        wm.process();
        vTaskDelay(pdMS_TO_TICKS(25));
        if (millis() - portalStart > 300000){
          break;
        }
      }

      bool ok = (WiFi.status() == WL_CONNECTED);
      wifiWasConnected = ok;
      if (ok){
        xEventGroupSetBits(args->flags, EG_WIFI_OK);
        setupTime(args->flags);     // NTP setelah set WiFi baru
        showToast("WiFi OK");
      } else {
        xEventGroupClearBits(args->flags, EG_WIFI_OK);
        xEventGroupClearBits(args->flags, EG_TIME_OK);
        showToast("WiFi Fail");
      }

      WiFi.mode(WIFI_STA);
    }

    // ====== Monitor status WiFi biasa (tanpa portal) ======
    bool wifiNow = (WiFi.status() == WL_CONNECTED);

    if (wifiNow && !wifiWasConnected){
      // Baru saja connect
      wifiWasConnected = true;
      xEventGroupSetBits(args->flags, EG_WIFI_OK);
      xEventGroupClearBits(args->flags, EG_TIME_OK); // pastikan re-sync
      setupTime(args->flags);                        // <-- ini yang dulu tidak pernah dipanggil
    } else if (!wifiNow && wifiWasConnected){
      // Baru saja putus
      wifiWasConnected = false;
      xEventGroupClearBits(args->flags, EG_WIFI_OK);
      xEventGroupClearBits(args->flags, EG_TIME_OK);
    }

    EventBits_t flags = xEventGroupGetBits(args->flags);

    // ====== Kirim data ke server ======
    if (millis() - lastPost >= POST_INTERVAL_MS){
      lastPost = millis();
      if (xQueueReceive(args->qTelemetry, &snap, 0)==pdTRUE){

        // WiFi belum OK → tidak usah POST
        if (!(flags & EG_WIFI_OK)){
          strncpy(args->status->postStatus, "WiFi--",
                  sizeof(args->status->postStatus)-1);
          args->status->postStatus[sizeof(args->status->postStatus)-1] = '\0';
          vTaskDelay(pdMS_TO_TICKS(200));
          continue;
        }

        // NTP belum sync → jangan POST dulu
        if (!(flags & EG_TIME_OK)){
          strncpy(args->status->postStatus, "WAITNTP",
                  sizeof(args->status->postStatus)-1);
          args->status->postStatus[sizeof(args->status->postStatus)-1] = '\0';
          vTaskDelay(pdMS_TO_TICKS(200));
          continue;
        }

        // NTP sudah OK → boleh POST
        postData(snap, *(args->status));
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// void TaskHTTP(void* param){
//   auto* args = static_cast<TaskHTTPArgs*>(param);
//   if (!args || !args->qTelemetry || !args->status || !args->flags){
//     vTaskDelete(NULL);
//     return;
//   }

//   WiFi.mode(WIFI_STA);
//   WiFi.begin();

//   WiFiManager wm;
//   wm.setConfigPortalTimeout(300);
//   wm.setConfigPortalBlocking(false);

//   uint32_t lastPost = 0;
//   DisplayData snap;

//   for(;;){
//     EventBits_t f = xEventGroupGetBits(args->flags);
//     if (f & EG_PORTAL_ON){
//       xEventGroupClearBits(args->flags, EG_PORTAL_ON);

//       WiFi.disconnect(true, true);
//       vTaskDelay(pdMS_TO_TICKS(200));

//       wm.startConfigPortal("Aerasea-Setup");
//       uint32_t portalStart = millis();
//       while (wm.getConfigPortalActive()){
//         wm.process();
//         vTaskDelay(pdMS_TO_TICKS(25));
//         if (millis() - portalStart > 300000){
//           break;
//         }
//       }

//       bool ok = (WiFi.status() == WL_CONNECTED);
//       if (ok){
//         xEventGroupSetBits(args->flags, EG_WIFI_OK);
//         setupTime(args->flags);
//         showToast("WiFi OK");
//       } else {
//         xEventGroupClearBits(args->flags, EG_WIFI_OK);
//         showToast("WiFi Fail");
//       }

//       WiFi.mode(WIFI_STA);
//     }

//     if (WiFi.status()==WL_CONNECTED) xEventGroupSetBits(args->flags, EG_WIFI_OK);
//     else                             xEventGroupClearBits(args->flags, EG_WIFI_OK);

//     EventBits_t flags = xEventGroupGetBits(args->flags);

//     // Heuristik tambahan: kalau jam sistem sudah > 2023, anggap NTP OK
//     if (!(flags & EG_TIME_OK)) {
//       time_t now; time(&now);
//       if (now > 1700000000) {
//         xEventGroupSetBits(args->flags, EG_TIME_OK);
//         flags |= EG_TIME_OK;
//       }
//     }

//     if (millis() - lastPost >= POST_INTERVAL_MS){
//       lastPost = millis();
//       if (xQueueReceive(args->qTelemetry, &snap, 0)==pdTRUE){

//         // *** PENTING: jangan POST sebelum NTP OK ***
//         if (!(flags & EG_TIME_OK)) {
//           // Tampilkan status di OLED: POST:WAITNTP
//           strncpy(args->status->postStatus, "WAITNTP",
//                   sizeof(args->status->postStatus)-1);
//           args->status->postStatus[sizeof(args->status->postStatus)-1] = '\0';
//           continue; // skip sampai waktu valid
//         }

//         // Kalau NTP sudah OK, baru boleh kirim data
//         if (WiFi.status()==WL_CONNECTED){
//           postData(snap, *(args->status));
//         }
//       }
//     }

//     vTaskDelay(pdMS_TO_TICKS(500));
//   }
// }

// =======================
// SETUP & LOOP
// =======================
void setup(){
  Serial.begin(115200);
  delay(100);

  ledInit();

  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(TFT_ROTATION);
  tft.invertDisplay(1);
  tft.setTextWrap(false);
  tft.fillScreen(TFT_BLACK);
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
#ifdef TFT_BACKLIGHT_ON
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
#else
  digitalWrite(TFT_BL, HIGH);
#endif
#endif

  analogReadResolution(12);
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);

  qInput     = xQueueCreate(16, sizeof(InputEvent));
  qDisplay   = xQueueCreate(1,  sizeof(DisplayData));
  qTelemetry = xQueueCreate(1,  sizeof(DisplayData));
  qCalib     = xQueueCreate(4,  sizeof(CalibMsg));
  mRS485     = xSemaphoreCreateMutex();
  egFlags    = xEventGroupCreate();

  gInputArgs.qInput = qInput;

  gUIArgs.qInput   = qInput;
  gUIArgs.qDisplay = qDisplay;
  gUIArgs.qCalib   = qCalib;
  gUIArgs.flags    = egFlags;
  gSensorArgs.qDisplay   = qDisplay;
  gSensorArgs.qTelemetry = qTelemetry;
  gSensorArgs.qCalib     = qCalib;
  gSensorArgs.rs485      = mRS485;
  gSensorArgs.flags      = egFlags;
  gSensorArgs.status     = &gStatus;
  gSensorArgs.ds         = &dsAsync;

  gHTTPArgs.qTelemetry = qTelemetry;
  gHTTPArgs.flags      = egFlags;
  gHTTPArgs.status     = &gStatus;

  xTaskCreatePinnedToCore(TaskInput,   "TaskInput",   4096, &gInputArgs,   5, NULL, 1);
  xTaskCreatePinnedToCore(TaskUI,      "TaskUI",      6144, &gUIArgs,      4, NULL, 1);
  xTaskCreatePinnedToCore(TaskSensors, "TaskSensors", 6144, &gSensorArgs,  1, NULL, 0);
  xTaskCreatePinnedToCore(TaskHTTP,    "TaskHTTP",    6144, &gHTTPArgs,    1, NULL, 0);
}

void loop(){
  vTaskDelay(pdMS_TO_TICKS(1000));
}
