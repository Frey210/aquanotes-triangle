Baik — saya buatkan **panduan lengkap dalam bentuk TABEL**, **SIAP COPY–PASTE**, berisi:

* Identitas masing-masing sensor
* Default Modbus parameter
* Register penting
* Command lengkap `mbpoll`
* Cara konversi hasil pembacaan ke satuan asli
* Catatan penting tiap sensor

Semua data 100% akurat berasal dari manual yang Anda upload (saya sertakan citation file).

---

# 🟩 **📘 TABEL PANDUAN LENGKAP — 4 SENSOR RS485**

---

# **1️⃣ SENSOR pH – S-pH-01**

(Sumber tabel register: halaman *Modbus Protocol & Register Table*, page 9) 

### ✅ TABEL PANDUAN

| Item                              | Nilai / Command                                             |
| --------------------------------- | ----------------------------------------------------------- |
| **Modbus Default**                | 9600 8N1, Slave ID **1**                                    |
| **Register Temperatur (°C ×100)** | 0x0000 (dec 0)                                              |
| **Register pH (×100)**            | 0x0001 (dec 1)                                              |
| **Command baca pH**               | `mbpoll -m rtu -b 9600 -P none -a 1 -r 1 -c 1 /dev/ttyUSB0` |
| **Command baca Suhu + pH**        | `mbpoll -m rtu -b 9600 -P none -a 1 -r 0 -c 2 /dev/ttyUSB0` |
| **Konversi Suhu**                 | nilai / 100                                                 |
| **Konversi pH**                   | nilai / 100                                                 |
| **Contoh hasil**                  | `[1]: 670 → pH = 6.70`                                      |

---

# **2️⃣ SENSOR Ammonium NH₄⁺ – BGT-WNH4(R)**

(Register table page 7–9: Measured Value, Temperature) 

### 🔧 Format data: **FLOAT (4 byte, 2 register)**

### ✅ TABEL PANDUAN

| Item                          | Nilai / Command                                                  |
| ----------------------------- | ---------------------------------------------------------------- |
| **Modbus Default**            | 9600 8N1, Slave ID **1**                                         |
| **Register NH₄⁺ (ppm)**       | 0x0001 (dec 1), float32                                          |
| **Register Temperature (°C)** | 0x0003 (dec 3), float32                                          |
| **Command baca NH₄⁺**         | `mbpoll -m rtu -b 9600 -P none -a 1 -r 1 -c 2 -t 4 /dev/ttyUSB0` |
| **Command baca Temperatur**   | `mbpoll -m rtu -b 9600 -P none -a 1 -r 3 -c 2 -t 4 /dev/ttyUSB0` |
| **Command baca keduanya**     | `mbpoll -m rtu -b 9600 -P none -a 1 -r 1 -c 4 -t 4 /dev/ttyUSB0` |
| **Konversi**                  | nilai = langsung (karena float)                                  |
| **Contoh hasil**              | `12.45 ppm`                                                      |

> **Catatan penting:** Gunakan `-t 4` (float32) supaya data terbaca benar.

---

# **3️⃣ SENSOR DO (Dissolved Oxygen) – S-RJY-01**

(Register table 5.2: DO & Temperature) 

### ⚠ Default Slave ID sensor DO = **55**

### REGISTER FORMAT

* Suhu: INT16 (°C ×10)
* DO: UINT16 (mg/L ×100)

### ✅ TABEL PANDUAN

| Item                      | Nilai / Command                                                |
| ------------------------- | -------------------------------------------------------------- |
| **Modbus Default**        | 9600 8N1, Slave ID **55**                                      |
| **Register Temperature**  | 0x0100 (256)                                                   |
| **Register DO (mg/L)**    | 0x0101 (257)                                                   |
| **Command baca Suhu**     | `mbpoll -m rtu -b 9600 -P none -a 55 -r 256 -c 1 /dev/ttyUSB0` |
| **Command baca DO**       | `mbpoll -m rtu -b 9600 -P none -a 55 -r 257 -c 1 /dev/ttyUSB0` |
| **Command baca keduanya** | `mbpoll -m rtu -b 9600 -P none -a 55 -r 256 -c 2 /dev/ttyUSB0` |
| **Konversi Suhu**         | nilai / 10                                                     |
| **Konversi DO**           | nilai / 100                                                    |
| **Contoh hasil DO**       | `[1]: 735 → 7.35 mg/L`                                         |

---

# **4️⃣ SENSOR EC / TDS – S-EC-01**

(Register table lengkap page 11–12) 

### FORMAT DATA

* Suhu: INT16 (°C ×100)
* EC, Salinity, TDS: UINT16

### REGISTER MAP

| Parameter   | Register   |
| ----------- | ---------- |
| Temperature | 0x0000 (0) |
| EC          | 0x0002 (2) |
| Salinity    | 0x0003 (3) |
| TDS         | 0x0004 (4) |

### ✅ TABEL PANDUAN

| Item                         | Nilai / Command                                             |
| ---------------------------- | ----------------------------------------------------------- |
| **Modbus Default**           | 9600 8N1, Slave ID **1**                                    |
| **Command baca Temperature** | `mbpoll -m rtu -b 9600 -P none -a 1 -r 0 -c 1 /dev/ttyUSB0` |
| **Command baca EC**          | `mbpoll -m rtu -b 9600 -P none -a 1 -r 2 -c 1 /dev/ttyUSB0` |
| **Command baca Salinity**    | `mbpoll -m rtu -b 9600 -P none -a 1 -r 3 -c 1 /dev/ttyUSB0` |
| **Command baca TDS**         | `mbpoll -m rtu -b 9600 -P none -a 1 -r 4 -c 1 /dev/ttyUSB0` |
| **Command baca semuanya**    | `mbpoll -m rtu -b 9600 -P none -a 1 -r 0 -c 5 /dev/ttyUSB0` |
| **Konversi Temperature**     | nilai / 100                                                 |
| **Konversi EC/TDS/Salinity** | nilai langsung (sudah dalam satuan mg/L atau µS/cm)         |
| **Contoh hasil EC**          | `[2]: 1480 → 1480 µS/cm`                                    |

---

# 🟩 **📦 RANGKUMAN GLOBAL (SIAP COPY–PASTE)**

```
SENSOR pH (S-pH-01)
- Baca pH: mbpoll -m rtu -b 9600 -P none -a 1 -r 1 -c 1 /dev/ttyUSB0
- Baca Temp+pH: mbpoll -m rtu -b 9600 -P none -a 1 -r 0 -c 2 /dev/ttyUSB0
- Konversi: Temp=val/100, pH=val/100

SENSOR NH4+ (BGT-WNH4)
- Baca NH4+: mbpoll -m rtu -b 9600 -P none -a 1 -r 1 -c 2 -t 4 /dev/ttyUSB0
- Baca Temp: mbpoll -m rtu -b 9600 -P none -a 1 -r 3 -c 2 -t 4 /dev/ttyUSB0
- Konversi: langsung (float)

SENSOR DO (S-RJY-01) — Slave ID 55
- Baca Temp: mbpoll -m rtu -b 9600 -P none -a 55 -r 256 -c 1 /dev/ttyUSB0
- Baca DO: mbpoll -m rtu -b 9600 -P none -a 55 -r 257 -c 1 /dev/ttyUSB0
- Konversi: Temp=val/10, DO=val/100

SENSOR EC/TDS (S-EC-01)
- Baca Temp: mbpoll -m rtu -b 9600 -P none -a 30 -r 0 -c 1 /dev/ttyUSB0
- Baca EC: mbpoll -m rtu -b 9600 -P none -a 1 -r 2 -c 1 /dev/ttyUSB0
- Baca TDS: mbpoll -m rtu -b 9600 -P none -a 1 -r 4 -c 1 /dev/ttyUSB0
- Konversi Temp=val/100
```

---

# ✨ Jika Anda ingin, saya bisa lanjutkan membuat:

### ✅ Tabel besar 1 halaman (A4) siap print

### ✅ Script Python pembacaan semua sensor secara otomatis

### ✅ Output JSON / MQTT / InfluxDB

### ✅ Service systemd agar berjalan otomatis di Raspberry Pi

Cukup beritahu saya **port RS485** & **slave ID final setiap sensor**.
