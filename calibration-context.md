# RK500 Water Quality Sensor Calibration Specification

## Hardware

Device:

* RS485 Modbus RTU

Sensors:

* RK500-12 pH Sensor
* RK500-04 Fluorescence DO Sensor
* RK500-13 EC/TDS/Salinity Sensor

Communication:

* Modbus RTU RS485
* Floating Point IEEE754
* Display format = Float Inverse (Modbus Poll compatible)

---

# 1. EC / TDS / Salinity Sensor Calibration

## Calibration Method

Sensor supports:

* One-Click Calibration via RS485

## Standard Solutions

### EC Low Range

Solution:

```text
1413 µS/cm
```

Equivalent:

```text
1.413 mS/cm
```

Recommended volume:

```text
30-50 mL
```

---

### EC High Range

Solution:

```text
12.88 mS/cm
```

Recommended volume:

```text
30-50 mL
```

---

## Calibration Procedure

1. Rinse probe with DI water.
2. Place probe into standard solution.
3. Wait 2-5 minutes until stable.
4. Send calibration value via Modbus.
5. Verify reading.

---

## Modbus Command

Register:

```text
0x50
(decimal 80)
```

Function:

```text
0x10
Write Multiple Registers
```

Quantity:

```text
2 Registers
```

Format:

```text
Float IEEE754
```

### Example 1413 µS/cm

Send:

```text
1.413
```

NOT:

```text
1413
```

because sensor default unit is:

```text
mS/cm
```

---

## UI Flow

```text
Calibration
 └─ EC Calibration
      ├─ 1413 µS/cm
      └─ 12.88 mS/cm
```

When selected:

```text
Insert sensor into solution
Wait until stable
Press ENTER
```

System:

```text
Write Float to Register 0x50
Show Success/Failed
```

---

# 2. pH Sensor Calibration

## Supported Calibration Points

Fixed calibration only:

```text
pH 4.00
pH 7.00
pH 10.00
```

Recommended:

```text
2-point calibration
pH7 + pH4

or

3-point calibration
pH4 + pH7 + pH10
```

---

## Calibration Solutions

Volume:

```text
30-50 mL each
```

Buffers:

```text
pH 4.00
pH 7.00
pH 10.00
```

---

## Fixed Calibration Commands

Register:

```text
0x55
(decimal 85)
```

Function:

```text
0x06
Write Single Register
```

---

### pH 4.00

Send:

```text
03 06 00 55 00 04
```

Value:

```text
4
```

---

### pH 7.00

Send:

```text
03 06 00 55 00 07
```

Value:

```text
7
```

---

### pH 10.00

Send:

```text
03 06 00 55 00 0A
```

Value:

```text
10
```

---

## Calibration Procedure

1. Rinse electrode.
2. Place in pH7 buffer.
3. Wait stable.
4. Send calibration command.
5. Rinse.
6. Repeat for pH4.
7. Repeat for pH10 (optional).

---

## One-Click Calibration

Register:

```text
0x50
```

Function:

```text
0x10
Write Multiple Registers
```

Format:

```text
Float IEEE754
```

Example:

If actual solution:

```text
pH 4.46
```

Send:

```text
4.46
```

as IEEE754 float.

---

## UI Flow

```text
Calibration
 └─ pH Calibration
      ├─ pH 7
      ├─ pH 4
      ├─ pH 10
      └─ Auto (One-Click)
```

---

# 3. Dissolved Oxygen Sensor Calibration

## Calibration Types

### Zero Oxygen Calibration

Solution:

```text
Na2SO3 (Sodium Sulfite)
```

Mix:

```text
5 g Na2SO3
100 mL DI Water
```

or

```text
2.5 g Na2SO3
50 mL DI Water
```

Recommended kit:

```text
5 g sachet
```

---

### Air Calibration

No liquid required.

Use:

```text
Ambient Air
```

Sensor clean and dry.

---

## Zero Calibration Procedure

1. Dissolve Na2SO3.
2. Place probe into solution.
3. Wait 3 minutes.
4. Execute Zero Calibration.
5. Verify reading near:

```text
0.00 mg/L
```

---

## Air Calibration Procedure

1. Remove sensor.
2. Rinse.
3. Dry water droplets.
4. Expose to air.
5. Wait 180 seconds.
6. Execute Air Calibration.

Expected:

```text
100% Saturation
```

---

## UI Flow

```text
Calibration
 └─ DO Calibration
      ├─ Zero Oxygen
      └─ Air Saturation
```

---

# Recommended Calibration Kit Included With Product

## Liquids

| Item           | Volume |
| -------------- | ------ |
| pH4            | 50 mL  |
| pH7            | 50 mL  |
| pH10           | 50 mL  |
| EC 1413 µS/cm  | 50 mL  |
| EC 12.88 mS/cm | 50 mL  |

---

## Powder

| Item   | Quantity   |
| ------ | ---------- |
| Na2SO3 | 5 g sachet |

---

# Suggested TFT Menu Structure

```text
Main Menu
│
├── Live Monitoring
│
├── Calibration
│   │
│   ├── pH
│   │   ├── pH 7
│   │   ├── pH 4
│   │   ├── pH 10
│   │   └── Auto
│   │
│   ├── EC
│   │   ├── 1413 µS/cm
│   │   └── 12.88 mS/cm
│   │
│   └── DO
│       ├── Zero Oxygen
│       └── Air Saturation
│
└── Settings
```

Untuk implementasi yang lebih profesional, saya juga menyarankan menambahkan **wizard kalibrasi bertahap** dengan status:

```text
Step 1/3
Insert sensor into calibration solution

[ENTER] Continue
```

```text
Step 2/3
Waiting stability...
Current: 1.397 mS/cm

[████████░░]
```

```text
Step 3/3
Calibration Success

New Value: 1.413 mS/cm

[ENTER]
```

karena jauh lebih ramah pengguna dibanding menu kalibrasi yang langsung menulis register Modbus.
