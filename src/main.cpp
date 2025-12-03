/*
 * Aerasea RTOS UI – ESP32-S3
 * Versi: 4 tombol (UP, DOWN, OK, BACK), tanpa encoder
 * Display: OLED 0.9" SSD1306 128x64 (I2C)
 * PlatformIO Version
 */

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
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

// OLED driver: sekarang pakai SSD1306 (0.9" / 0.96")
// #define OLED_DRIVER_SH1106
#define OLED_DRIVER_SSD1306

// I2C
static const int SDA_PIN = 8;
static const int SCL_PIN = 9;
static const uint32_t I2C_FREQ = 400000;

// DS18B20
#define DS18B20_PIN 13

// RS485 (shared bus)
#define RS485_RX_PIN    16
#define RS485_TX_PIN    15
#define RS485_DE_RE_PIN 14

// Slave IDs (pH/EC/NH4 sekarang unik, DO tetap 55)
#define PH_SLAVE_ID   3   // pH
#define EC_SLAVE_ID   2   // EC/TDS/Sal
#define NH4_SLAVE_ID  1   // NH4
#define DO_SLAVE_ID   55  // DO (default)

// Serial format
#define PH_BAUD   9600
#define EC_BAUD   9600
#define NH4_BAUD  9600
#define DO_BAUD   9600

// Server & Identitas
const char* POST_URL   = "https://aeraseaku.inkubasistartupunhas.id/sensor/";
const char* UID        = "AER2023AQ0014";
const char* FW_VERSION = "v1.3.0-RTOS-BTN4";
const uint32_t POST_INTERVAL_MS = 10000; // 10 detik
const uint32_t HTTP_TIMEOUT_MS  = 3500;

// WiFi Manager & NTP
const char* NTP_SERVER  = "pool.ntp.org";
const long  GMT_OFFSET  = 8 * 3600; // GMT+8
const int   DAYLIGHT_OFF= 0;

// =======================
//  INPUT: 4 BUTTON
// =======================
// Button aktif LOW (INPUT_PULLUP)
#define BTN_UP     5
#define BTN_DOWN   6
#define BTN_OK     7
#define BTN_BACK   10

// =======================
//  OBJEK GLOBAL
// =======================
#ifdef OLED_DRIVER_SH1106
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
#endif

OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
HardwareSerial RS485(2);

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
  SensorMode* mode;
};

struct TaskSensorArgs {
  QueueHandle_t qDisplay;
  QueueHandle_t qTelemetry;
  QueueHandle_t qCalib;
  SemaphoreHandle_t rs485;
  EventGroupHandle_t flags;
  SensorMode* mode;
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
SensorMode gMode = SensorMode::PH;
RuntimeStatus gStatus;
AsyncDS18B20 dsAsync(ds18b20);

TaskInputArgs   gInputArgs{};
TaskUIArgs      gUIArgs{};
TaskSensorArgs  gSensorArgs{};
TaskHTTPArgs    gHTTPArgs{};

// Menu cursors
int menuCursor=0, modeCursor=0, calCursor=0, ecCalCursor=0, nh4CalCursor=0, doCalCursor=0, phCalCursor=0;

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
  configTime(GMT_OFFSET, DAYLIGHT_OFF, NTP_SERVER);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    xEventGroupSetBits(flags, EG_TIME_OK);
  } else {
    xEventGroupClearBits(flags, EG_TIME_OK);
  }
}

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
  "Kalibrasi Sensor",
  "Sensor Mode"
};
const int MAIN_COUNT = sizeof(MAIN_ITEMS)/sizeof(MAIN_ITEMS[0]);

const char* MODE_ITEMS[] = { "pH Sensor", "EC Sensor", "NH4 Sensor" };
const int MODE_COUNT = sizeof(MODE_ITEMS)/sizeof(MODE_ITEMS[0]);

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
void drawSplashFrame(uint8_t pct, bool wifiOK, bool ntpOK){
  uint8_t contrast = map(pct, 0, 100, 16, 255);
  u8g2.setContrast(contrast);

  u8g2.clearBuffer();
  int x = (128 - AERASEA_LOGO_WIDTH)  / 2;
  int y = (64  - AERASEA_LOGO_HEIGHT) / 2 - 6;
  u8g2.drawXBMP(x, y, AERASEA_LOGO_WIDTH, AERASEA_LOGO_HEIGHT, aerasea_logo_128x64_inverted_bits);

  u8g2.setFont(u8g2_font_6x12_tf);
  char ln1[28]; snprintf(ln1, sizeof(ln1), "UID: %s", UID);
  char ln2[20]; snprintf(ln2, sizeof(ln2), "FW : %s", FW_VERSION);
  u8g2.drawStr(0, 54, ln1);
  u8g2.drawStr(0, 64, ln2);

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(80, 54, wifiOK ? "WiFi:OK" : "WiFi:--");
  u8g2.drawStr(80, 63, ntpOK  ? "NTP:OK"  : "NTP:--");

  u8g2.sendBuffer();
}

void drawDashboard(const DisplayData& d){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  char buf[32];

  snprintf(buf, sizeof(buf), "T: %.2f C", d.t_ds);
  u8g2.drawStr(0, 10, buf);

  if (d.do_ok) snprintf(buf, sizeof(buf), "DO: %.2f mg/L", d.do_mgL);
  else         snprintf(buf, sizeof(buf), "DO: ---");
  u8g2.drawStr(0, 22, buf);

  if (d.ph_ok) snprintf(buf, sizeof(buf), "pH: %.2f", d.ph);
  else         snprintf(buf, sizeof(buf), "pH: ---");
  u8g2.drawStr(0, 34, buf);

  if (d.ec_ok) snprintf(buf, sizeof(buf), "EC: %.0f uS/cm", d.ec);
  else         snprintf(buf, sizeof(buf), "EC: ---");
  u8g2.drawStr(0, 46, buf);

  if (d.nh4_ok) snprintf(buf, sizeof(buf), "NH4: %.2f mg/L", d.nh4);
  else          snprintf(buf, sizeof(buf), "NH4: ---");
  u8g2.drawStr(0, 58, buf);

  u8g2.setFont(u8g2_font_5x8_tr);
  snprintf(buf, sizeof(buf), "WiFi:%s NTP:%s POST:%s",
           d.wifiOK ? "OK":"--",
           d.ntpOK  ? "OK":"--",
           d.postStatus);
  u8g2.drawStr(0, 64, buf);

  if (toastActive()){
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 10);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(2, 8, toastMsg);
    u8g2.setDrawColor(1);
  }

  u8g2.sendBuffer();
}

void drawMenu(const char* title, const char* items[], int nItems, int cursor){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  u8g2.drawStr(0, 10, title);
  u8g2.drawHLine(0, 12, 128);

  int y = 24;
  for (int i = 0; i < nItems; ++i){
    if (i == cursor){
      u8g2.drawBox(0, y-9, 128, 11);
      u8g2.setDrawColor(0);
      u8g2.drawStr(2, y, items[i]);
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(2, y, items[i]);
    }
    y += 12;
  }

  if (toastActive()){
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 10);
    u8g2.setDrawColor(0);
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(2, 8, toastMsg);
    u8g2.setDrawColor(1);
  }

  u8g2.sendBuffer();
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
  if (!args || !args->qInput || !args->qDisplay || !args->mode || !args->flags || !args->qCalib){
    vTaskDelete(NULL);
    return;
  }

  DisplayData disp = {};
  uint8_t splashPct = 0;
  uint32_t tSplash = millis();
  UIState ui = UIState::DASHBOARD;

  for(;;){
    InputEvent ev;
    while (xQueueReceive(args->qInput, &ev, 0) == pdTRUE){
      if (ev.type == ENC_DELTA){
        int dir = (ev.value > 0) ? 1 : -1;
        switch (ui){
          case UIState::MENU:
            menuCursor = constrain(menuCursor + dir, 0, MAIN_COUNT-1);
            break;
          case UIState::SENSOR_MODE:
            modeCursor = constrain(modeCursor + dir, 0, MODE_COUNT-1);
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
        if (ui==UIState::MENU || ui==UIState::WIFI_MGR || ui==UIState::SENSOR_MODE || ui==UIState::CALIB ||
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
              case 3: ui = UIState::SENSOR_MODE; break;
            }
          }
        } else if (ui == UIState::SENSOR_MODE){
          if (ev.value == BTN_OK){
            switch (modeCursor){
              case 0: *(args->mode) = SensorMode::PH;  showToast("Mode: pH");  break;
              case 1: *(args->mode) = SensorMode::EC;  showToast("Mode: EC");  break;
              case 2: *(args->mode) = SensorMode::NH4; showToast("Mode: NH4"); break;
            }
            ui = UIState::DASHBOARD;
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
      disp.mode = *(args->mode);
    }

    EventBits_t flags = xEventGroupGetBits(args->flags);
    bool wifiOK = (flags & EG_WIFI_OK);
    bool ntpOK  = (flags & EG_TIME_OK);

    uint32_t now = millis();
    if (now - tSplash < 2000){
      if (splashPct < 100 && now - tSplash > splashPct*10){
        splashPct++;
      }
      drawSplashFrame(splashPct, wifiOK, ntpOK);
    } else {
      switch (ui){
        case UIState::DASHBOARD:   drawDashboard(disp); break;
        case UIState::MENU:        drawMenu("Main Menu", MAIN_ITEMS, MAIN_COUNT, menuCursor); break;
        case UIState::SENSOR_MODE: drawMenu("Sensor Mode", MODE_ITEMS, MODE_COUNT, modeCursor); break;
        case UIState::CALIB:       drawMenu("Kalibrasi Sensor", CAL_ITEMS, CAL_COUNT, calCursor); break;
        case UIState::CAL_EC:      drawMenu("Cal EC (Auto)", EC_CAL_ITEMS, EC_CAL_COUNT, ecCalCursor); break;
        case UIState::CAL_NH4:     drawMenu("Cal NH4", NH4_CAL_ITEMS, NH4_CAL_COUNT, nh4CalCursor); break;
        case UIState::CAL_DO:      drawMenu("Cal DO", DO_CAL_ITEMS, DO_CAL_COUNT, doCalCursor); break;
        case UIState::CAL_PH:      drawMenu("Cal pH (S-PH-01)", PH_CAL_ITEMS, PH_CAL_COUNT, phCalCursor); break;
        case UIState::WIFI_MGR:    drawMenu("WiFi Manager", WIFI_MGR_ITEMS, WIFI_MGR_COUNT, 0); break;
      }
    }

    if (!toastActive()){
      clearToast();
    }

    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

// =======================
// TaskSensors (Core 0)
// =======================
void TaskSensors(void* param){
  auto* args = static_cast<TaskSensorArgs*>(param);
  if (!args || !args->qDisplay || !args->qTelemetry || !args->qCalib || !args->rs485 || !args->mode || !args->status || !args->ds || !args->flags){
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
  bindNode(mb_ph,  PH_SLAVE_ID);
  bindNode(mb_ec,  EC_SLAVE_ID);
  bindNode(mb_nh4, NH4_SLAVE_ID);
  bindNode(mb_do,  DO_SLAVE_ID);

  DisplayData cur = {};
  uint8_t slot = 0;
  const TickType_t rsLockTimeout = pdMS_TO_TICKS(50);

  for(;;){
    // proses perintah kalibrasi jika ada
    CalibMsg msg;
    while (xQueueReceive(args->qCalib, &msg, 0) == pdTRUE){
      if (xSemaphoreTake(args->rs485, rsLockTimeout) == pdTRUE){
        handleCalibration(msg.cmd, cur);
        xSemaphoreGive(args->rs485);
      }
    }

    ds.tick();
    float t = ds.temperatureC();
    if (!isnan(t)){
      cur.t_ds = t;
    }

    if (xSemaphoreTake(args->rs485, rsLockTimeout) == pdTRUE){
      switch (slot){
        case 0: { // DO (ID 55)
          float d_mg, d_t; bool ok = readDO(d_mg, d_t);
          cur.do_ok = ok; if (ok){ cur.do_mgL = d_mg; cur.do_tC = d_t; }
        } break;

        case 1: { // pH (PH_SLAVE_ID)
          float tp, v; bool ok = readPH(tp, v);
          cur.ph_ok = ok; if (ok){ cur.phT = tp; cur.ph = v; }
        } break;

        case 2: { // EC/TDS/Sal (EC_SLAVE_ID)
          float tec, ec, tds, sal; bool ok = readEC(tec, ec, tds, sal);
          cur.ec_ok = ok; if (ok){ cur.ecT = tec; cur.ec = ec; cur.tds = tds; cur.sal = sal; }
        } break;

        case 3: { // NH4 (NH4_SLAVE_ID)
          float nh4, tn; bool ok = readNH4(nh4, tn);
          cur.nh4_ok = ok; if (ok){ cur.nh4 = nh4; cur.nh4T = tn; }
        } break;
      }
      rs485Receive();
      xSemaphoreGive(args->rs485);
    }

    vTaskDelay(pdMS_TO_TICKS(1));

    slot = (slot + 1) & 0x03;

    cur.wifiOK = (WiFi.status()==WL_CONNECTED);
    EventBits_t flags = xEventGroupGetBits(args->flags);
    cur.ntpOK  = (flags & EG_TIME_OK);
    strncpy(cur.postStatus, args->status->postStatus, sizeof(cur.postStatus)-1);
    cur.postStatus[sizeof(cur.postStatus)-1] = '\0';
    cur.mode = *(args->mode);

    xQueueOverwrite(args->qDisplay, &cur);
    xQueueOverwrite(args->qTelemetry, &cur);

    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

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
  WiFi.begin();

  WiFiManager wm;
  wm.setConfigPortalTimeout(300);
  wm.setConfigPortalBlocking(false);

  uint32_t lastPost = 0;
  DisplayData snap;

  for(;;){
    EventBits_t f = xEventGroupGetBits(args->flags);
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
      if (ok){
        xEventGroupSetBits(args->flags, EG_WIFI_OK);
        setupTime(args->flags);
        showToast("WiFi OK");
      } else {
        xEventGroupClearBits(args->flags, EG_WIFI_OK);
        showToast("WiFi Fail");
      }

      WiFi.mode(WIFI_STA);
    }

    if (WiFi.status()==WL_CONNECTED) xEventGroupSetBits(args->flags, EG_WIFI_OK);
    else                             xEventGroupClearBits(args->flags, EG_WIFI_OK);

    EventBits_t flags = xEventGroupGetBits(args->flags);
    if (!(flags & EG_TIME_OK)){
      time_t now; time(&now);
      if (now > 1700000000) xEventGroupSetBits(args->flags, EG_TIME_OK);
    }

    if (millis() - lastPost >= POST_INTERVAL_MS){
      lastPost = millis();
      if (xQueueReceive(args->qTelemetry, &snap, 0)==pdTRUE){
        if (WiFi.status()==WL_CONNECTED){
          postData(snap, *(args->status));
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// =======================
// SETUP & LOOP
// =======================
void setup(){
  Serial.begin(115200);
  delay(100);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.sendBuffer();

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
  gUIArgs.mode     = &gMode;

  gSensorArgs.qDisplay   = qDisplay;
  gSensorArgs.qTelemetry = qTelemetry;
  gSensorArgs.qCalib     = qCalib;
  gSensorArgs.rs485      = mRS485;
  gSensorArgs.flags      = egFlags;
  gSensorArgs.mode       = &gMode;
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
