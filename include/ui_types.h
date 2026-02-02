#pragma once
#include <stdint.h>

// State mesin UI
enum class UIState : uint8_t{
  DASHBOARD,
  MENU,
  WIFI_MGR,
  CALIB,
  CAL_EC,
  CAL_NH4,
  CAL_DO,
  CAL_PH
};

// Snapshot data yang dikirim dari TaskSensors ke TaskUI/TaskHTTP
struct DisplayData {
  float t_ds;              // Suhu dari DS18B20

  float ph,  phT;          // Nilai pH dan suhu internal pH
  float ec,  tds, sal, ecT;// EC/TDS/Sal dan suhu internal EC
  float nh4, nh4T;         // NH4 dan suhunya
  float do_mgL, do_tC;     // DO dan suhu DO

  bool  ph_ok;             // true kalau pembacaan pH sukses
  bool  ec_ok;             // true kalau pembacaan EC sukses
  bool  nh4_ok;            // true kalau pembacaan NH4 sukses
  bool  do_ok;             // true kalau pembacaan DO sukses

  float bat_v;             // Tegangan baterai (V)
  float bat_pct;           // Persentase baterai (0-100)
  bool  bat_ok;            // true kalau pembacaan baterai valid

  bool  wifiOK;            // Status WiFi
  bool  ntpOK;             // Status NTP/time
  char  postStatus[16];    // Status HTTP POST terakhir ("OK", "E500", dsb)
};
