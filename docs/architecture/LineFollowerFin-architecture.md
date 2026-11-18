# Dokumentasi Teknis Arsitektur LineFollowerFin

## 1. Ikhtisar

**LineFollowerFin** adalah robot pelacak garis otonom untuk kompetisi uKit Explore 2, dirancang menggunakan Arduino Mega2560 dan library uKit Explore 2. Robot ini melacak jalur hitam di atas latar belakang putih menggunakan sensor IR, menerapkan kontrol PID, dan dilengkapi dengan fitur-fitur kompetisi canggih seperti prediction mode berbasis waypoint dan adaptive threshold.

**Arsitektur:** Modular berorientasi objek dengan pemisahan yang jelas antara:
- **Kontrol loop utama** (loop() di LineFollowerFin.ino)
- **Komponen inti** (sensor, kontrol PID, keamanan)
- **Fasilitas opsional** (Bluetooth, RC, Fuzzy PID, Heading Correction, Path Planner)
- **Persistensi** (EEPROM untuk tuning)
- **Kontrol & monitoring** (antarmuka dan telemetri)

## 2. File & Direktori

```
LineFollowerFin/
├── LineFollowerFin.ino              # Skets utama (15KB)
├── config.h                         # Semua konfigurasi (14.5KB)
├── config.cpp                       # Utility EEPROM
├── fwd.h                            # Forward declarations (460B)
├── status.h                         # Status jalan & state machine (750B)
├── sensor_garis.h/.cpp             # Sensor IR + filter + kalibrasi (1.8KB/.cpp 8.4KB)
├── penggerak.h/.cpp                # PID controller + output servo (2.9KB/.cpp 11.9KB)
├── pengaman.h/.cpp                 # Keselamatan: baterai, miring, halangan, macet (1.1KB/.cpp 4.4KB)
├── antarmuka.h/.cpp                 # UI: tombol, tepuk, serial command (1.3KB/.cpp 27.0KB)
├── mapping.h/.cpp                   # Waypoint memory + racing prediction (1.4KB/.cpp 3.8KB)
├── lap_timer.h/.cpp                # Statistik lap & timing (2.1KB/.cpp 5.2KB)
├── bluetooth_ctrl.h/.cpp           # Bluetooth control + telemetry (2.5KB/.cpp 11.8KB)
├── rc_ctrl.h/.cpp                   # Remote control mode (2.3KB/.cpp 6.0KB)
├── fuzzy_pid.h/.cpp                # Adaptive PID gains (1.3KB/.cpp 6.3KB)
├── heading_tracker.h/.cpp          # IMU heading correction (1.7KB/.cpp 3.0KB)
├── path_planner.h/.cpp             # A* path planning (4.0KB/.cpp 15.4KB)
├── tests/                          # Uji aplikasi
├── docs/                           # Dokumentasi (rincian pada subfolder)
├── docs/architecture/LineFollowerFin-architecture.md
├── docs/architecture/modules/       # Dokumentasi per modul
├── docs/api/                       # Dokumentasi API per modul
└── docs/design-decisions/          # Keputusan desain & alasan
```

## 3. Struktur Kontrol & State Machine

### 3.1 Loop Utama (LineFollowerFin.ino:loop())

**Urutan eksekusi** (hanya saat tidak ada error):

1. **protocol()** - mengawasi protokol pemindaian uKit (reset, start, stop, tune)
2. **Keamanan utama**:
   - baterai kritis → kembali
   - IMU miring/terangkat → kembali
3. **Kontrol UI**: `ui.cekTombol()`, `ui.cekTepuk()`, `ui.cekSerial()` (jika diaktifkan)
4. **Lampu otomatis** `aman.lampuOtomatis()` (throttle 200ms)
5. **Heading correction** `gerak.updateHeading()` (jika diaktifkan)
6. **Bluetooth/RC update** (jika diaktifkan)

### 3.2 Loop Kontrol Drive (status.jalan == true)

**Prioritas tertinggi**: ultrasonic halangan → kembali

**Priority urutan**:
1. **RACING prediction** → tampilkan aksi di persimpangan ambigu
2. **MODE RC** → ambil alih sepenuhnya
3. **SUBSTATE_INTER_CREEP** → dorong pelan melewati persimpangan (150ms)
4. **SUBSTATE_TURNING** → belok 90° non-blocking (250ms + timeout BELOK_TIMEOUT_MS)
5. **KONDISI A** – histeresis: dua loop berturut-turut sensor di garis → persimpangan
6. **KONDISI B** – LOST LINE RECOVERY:
   - Mulai timer pemulihan (2000ms)
   - >2000ms → berhenti + LED merah
   - ≤2000ms → spin mencari garis (putarCariGaris())
7. **KONDISI C** – MODE TRACER NORMAL dengan adaptive speed:
   - Deteksi macet → kembali
   - Reset integral setelah recovery exit
   - Loop PID di interval PID_INTERVAL_MS (10ms)
   - Update adaptive threshold (jika diaktifkan)
   - Update lap timer (jika diaktifkan)

### 3.3 Enum Kontrol

```cpp
enum SubState { SUBSTATE_TRACKING, SUBSTATE_INTER_CREEP, SUBSTATE_TURNING };
enum ModeOperasi {
  MODE_NORMAL,
  MODE_LEARNING,   // mode pembelajaran untuk waypoint
  MODE_RACING,     // prediksi berdasarkan waypoint
  MODE_RC          // remote control
};
enum EventType { EVENT_NONE, EVENT_INTERSECTION, EVENT_FINISH };
enum ActionType {
  ACT_NONE,
  ACT_STRAIGHT,
  ACT_LEFT,
  ACT_RIGHT
};
```

### 3.4 Manajer State (struct Status)

```cpp
struct Status {
  bool jalan = false;
  bool modeSetelKd = false;
  unsigned long recoveryMulai = 0;   // timer pemulihan
  ModeOperasi modeOperasi = MODE_NORMAL;
  SubState subState = SUBSTATE_TRACKING;
  unsigned long subStateMulai = 0;   // waktu mulai substate sekarang
  ActionType subStateAksi = ACT_NONE;
  int targetAmbangBelok = 0;        // ambang tujuan untuk belok
  bool dalamGaris = true;            // true jika satu sensor ≥ threshold
};
```

## 4. Komponen Inti

### 4.1 SensorGaris (sensor_garis.h/.cpp)

**Tanggung jawab:**
- Read sensor IR: kiri (ID 2), kanan (ID 1)
- Filter eksponensial untuk redam noise (ALFA_SENSOR)
- Adaptive threshold dinamis (jika diaktifkan)
- Kalibrasi otomatis & manual (EEPROM)
- Persistensi: simpan ambang kiri/kanan, Kp/Ki/Kd, kecepatanDasarMaks

**Method utama:**
- `bacaKiri()`, `bacaKanan()` – baca filter, throttle baca
- `kalibrasi()` – spin 360° (manual/oto) hitung min/max
- `simpan()`, `muat()` – EEPROM CRUD
- `updateAmbangDinamis()` – naikkan ambang sesuai kecepatan

**Cara kerja:**
```cpp
int bacaHalus(int id) {
  float filt = id == 1 ? filtKiri : filtKanan;
  if (filt < 0) return filt = bacaHalus(id); // inisialisasi
  return filt = ALPHA * nilaiBaru + (1 - ALPHA) * filt;
}
```

### 4.2 Penggerak (penggerak.h/.cpp)

**Tanggung jawab:** PID controller + output servo (derate 0-150, deadband ~25, slew ±14/loop)

**Method utama:**
- `hitung()` – loop PID, return true jika interval PID selesai
- `terapkanTerakhir()` – pertahankan speed terakhir saat PID jeda
- `jalankanArah()` – kecepatan bertanda untuk RC
- `putarCariGaris()` – recovery saat hilang garis
- `resetIntegralSetelahRecovery()` – buang hutang integral

**State PID:** error, lastError, I (dijepit ±50), P, D, kecepatan target kiri/kanan

**Gain Scheduling** (opsional): Kp/Ki/Kd berbeda saat speed rendah/mid/high

### 4.3 Pengaman (pengaman.h/.cpp)

**Tanggung jawab:** Deteksi situasi berbahaya, berhenti otomatis, lampu LED

**Method utama:**
- `bateraiKritis()` – voltase ≤ 5.5V (latch berhenti)
- `miring()` – roll/pitch ≥ 45° (throttle CEK_IMU_MS)
- `halangan()` – ultrasonic ≤ JARAK_BERHENTI_CM (100ms throttle)
- `macet()` – IR beku + magnitude accel stabil (3 detik)
- `lampuOtomatis()` – nyala saat gelap (<100 lux)

**Ketahanan:** Semua cek punya throttle untuk mencegah overload CPU.

### 4.4 Antarmuka (antarmuka.h/.cpp)

**Tanggung jawab:** Kontrol manusia: tombol, tepuk, serial command

**Method utama:**
- `cekTombol()` – non-blocking, cooldown COOLDOWN_TOMBOL_MS (300ms)
- `cekTepuk()` – toggle jalan/berhenti via suara (bystander)
- `cekSerial()` – parsing command (Kp/Ki/Kd/speed/ambang/calibrate/save/help)
- `tuningSequence()` – combo tombol B1+B2 untuk auto-tune relay

**Tombol kerja:**
- Tombol pendek: switch Kp/Kd tergantung modeSetelKd
- Tombol panjang: berhenti (jika jalan), simpan EEPROM + start (jika berhenti)
- Kombinasi tombol: kalibrasi otomatis 360° berbasis IMU

### 4.5 Mapper (mapping.h/.cpp)

**Tanggung jawab:** Memory waypoint & racing prediction

**Method utama:**
- `recordEvent()` – mode pembelajaran: simpan EVENT_INTERSECTION/EVENT_FINISH + ActionType
- `checkPrediction()` – berdasarkan offset waktu waypoint, return EventType & ActionType
- `simpan()` / `muat()` – EEPROM blok 24-266 (30 waypoint x 8 bytes)

**Tipe event/action:**
```cpp
EVENT_INTERSECTION → ACT_STRAIGHT/ACT_LEFT/ACT_RIGHT
EVENT_FINISH → ACT_NONE (lap selesai)
```

### 4.6 LapTimer (lap_timer.h/.cpp)

**Tanggung jawab:** Hitung waktu & statistik lap (opsional)

**Method utama:**
- `update()` – deteksi start/finish (kedua sensor ≥ THRESHOLD), hitung error/time avg/max
- `printCSV()` – output telemetri di interval TELEM_MS

**Storage:** Maks 15 laps (MAX_LAPS) di RAM

## 5. Fitur Opsional

### 5.1 Bluetooth (ENABLE_BLUETOOTH)

**Fungsi:** Serial Bluetooth (HM-10/ESP32) atau built-in uKit BT
**Protocol:** Dump CSV telemetri (interval BT_TELEMETRY_MS)
**Method utama:** `update()`, `sendTelemetry()`, `setCommandCallback()`

### 5.2 Remote Control (ENABLE_RC)

**Fungsi:** PWM/PPM (dua channel) atau Serial ESP-based
**Method utama:** `update()`, `getSpeeds()` → terapkan ke motor
**Feature:** hilang sinyal → berhenti + LED merah

### 5.3 Fuzzy PID (ENABLE_FUZZY_PID)

**Fungsi:** Dynamic gain adjustment berdasarkan error & delta_error
**Input:** error (±10), delta_error (±10)
**Output:** delta Kp (±5), delta Kd (±2), delta Ki (±0.5)
**Method:** Mamdani inference + centroid defuzzifikasi

### 5.4 Heading Correction (ENABLE_HEADING_CORRECTION)

**Fungsi:** Yaw drift correction menggunakan gyro Z (gain HEADING_GAIN)
**Method:** Integrasi gyro + kalibrasi bias (duration HEADING_CALIB_MS)

### 5.5 Path Planner (ENABLE_PATH_PLANNER)

**Fungsi:** Manual grid-based path planning (A*)
**Scope:** Bukan navigasi otonom (tidak ada odometri)
**RAM:** ~94% ATMega2560 (biarkan off kecuali debugging)

## 6. EEPROM Layout

**Alokasi** (byte address):
| Alamat | Isi | Keterangan |
|---------|---------|-------------|
| 3 | `77` | Guard uKit: `Initialization()` hapus 0-20 jika != 77 |
| 0 | `123` | Magic/check validitas EEPROM |
| 4 | `Kp` (float) | PID proportional |
| 8 | `Ki` (float) | PID integral |
| 12 | `Kd` (float) | PID derivative |
| 16 | `kecepatanDasarMaks` (int) | Batas speed maksimal |
| 20 | `AMBANG_KIRI` (int) | Ambang sensor kiri |
| 22 | `AMBANG_KANAN` (int) | Ambang sensor kanan |
| 24-266 | Peta waypoint (30 x 8 bytes) | Blok MAP (
   - byte 0: magic 77 |
   - byte 1: count |
   - sisa: Waypoint struct (type + action + offset)
) |
| 280 | `HEADING_GAIN` (float) | Heading correction gain |
| 284 | `HEADING_TARGET` (float) | Target heading saved |
| 288-289 | `AMBANG_KIRI_BASE`/`AMBANG_KANAN_BASE` (int) | Dasar untuk adaptive threshold |
| 292 | `HEADING_BIAS` (float) | Bias gyro sudah dikalibrasi |
| 296 | `88` | Magic data heading |
| 297 | `66` | Magic data adaptive threshold |

**Catatan:** Guard byte 3 ditulis di `setup()` SEBELUM `Initialization()` agar tidak menghapus konfigurasi yang tersimpan.

## 7. Flow Kontrol & Sinyal

### 7. Среди Sinyal Sensor
```cpp
// IDs (config.h)
LIGHT_SENSOR_ID = 1        // Untuk lampu otomatis
def readLightValue(id) -> lux (0-4000)

def readInfraredDistance(2) -> cm (kiri)
def readInfraredDistance(1) -> cm (kanan)
def readBatteryVoltage() -> V (0-8.4)

def readUltrasonicDistance(id) -> cm (0-400)
def readSoundValue(id) -> intensitas (0-1023)
```

### 7.2 Method Controller → Motor
```cpp
// Output servo speed (0-255)
// Derate: 0-150 (output efectivo)
// Deadband: speed 1-24 → 25 (atau 0)
// Slew: target_prev + SLEW_MAKS_PER_LOOP

void jalankanArah(int speedKiri, int speedKanan) {
  gerak.jalankanArah(speedKiri, speedKanan);
}
```

### 7.3 Loop Timing

**Interval:** PID_INTERVAL_MS = 10ms (pastikan dt konstan)

**Throttles utama:**
- CEK_CAHAYA_MS = 200 (lampu gelap)
- CEK_HALANGAN_MS = 100 (ultrasonic)
- CEK_BATERAI_MS = 500 (voltase)
- CEK_IMU_MS = 50 (miring)
- TEPUK_COOLDOWN_MS = 800 (histeresis tepuk)

## 8. Penanganan Edge Case & Desain

### 8.1 Histeresis

**Tujuannya:** Mencegah false positive di error kecil.

**Impl:**
- Inter-sensor: butuh 2 loop berturut-turut sensor di garis → persimpangan (SUBSTATE_INTER_CREEP)
- Edge recovery: sensor di garis → mulai recovery (2000ms timeout)

### 8.2 Adaptive Threshold

**Mengapa:** Noise meningkat seiring kecepatan → naikkan ambang.

**Formula:** ambang = AMBANG_BASE + ADAPT_SPEED_FACTOR * speed

**Batas:** ADAPT_THRESH_MIN = 3, ADAPT_THRESH_MAX = 18

### 8.3 Integral Anti-Windup

**I term dijepit ±BATAS_INTEGRAL (50)** saat error kecil.

**Reset integral:** Setelah recovery exit (resetIntegralSetelahRecovery)

### 8.4 Priority Antar Mode Operasi

```
masa-jalan = true
├── halangan ultrasonik → kembali
├── modeOperasi = MODE_RC → RC mengambil alih (return)
├── modeOperasi = MODE_RACING → prediksi validasi
├── substate = SUBSTATE_INTER_CREEP → 150ms creep
├── substate = SUBSTATE_TURNING → 250ms + timeout BELOK_TIMEOUT_MS
└── else → tracer PID normal
```

### 8.5 Desain Decisions & Alasan

1. **Non-blocking UI:** Gunakan cooldown millis() bukan delay() → robot tetap bisa melacak garis saat UI merespon.
2. **Non-blocking turn:** Substate machine → belok 90° di 150-400ms, tidak block loop tracer.
3. **Adaptive threshold:** Redam noise di kecepatan tinggi tanpa tuning manual.
4. **Recovery timeout:** 2000ms → cegah spin tanpa tujuan di garis hilang.
5. **Ultrasonic prioritas:** Halangan depan → stop otomatis (keselamatan utama).
6. **EEPROM guard:** Byte 3 = 77 → cegah uKit Initialization() menghapus tuning yang tersimpan.
7. **No malloc:** Semua struktur statis (node pool, waypoint) → hindari stack overflow.

## 9. Protokol & Inisialisasi

### 9.1 Eksekusi protocol() (dari LoopFollowerFin.ino)

```cpp
void protocol() {
  if (protocolRunState) {  // command dari serial /uKit
    if (cmd == "start") {
      status.jalan = true;
      status.modeOperasi = MODE_NORMAL;
      lapTimer.begin();
      mapper.mulaiRun();  // reset baseline waktu balapan
    } else if (cmd == "stop") {
      status.jalan = false;
    } else if (cmd == "tune") {
      status.modeSetelKd = true;
    } else if (cmd == "help") { /* tampilkan daftar */ }
  }
}
```

### 9.2 Order Inisialisasi (setup())

```cpp
1. EEPROM.update(3, 77);       // guard
touchi.Initialization();       // library uKit: hapus 0-20 bila byte 3 != 77
IMU::init();

2. Jika protocolRunState == false (startup normal)
   sensor.muat();
   mapper.muat();  // waypoint learning
   jika EEPROM tidak valid + AUTO_CALIBRATE_ON_BOOT = true → sensor.kalibrasi()

3. lapTimer.begin();
   sensor.setAdaptiveThreshold() / gerak.setHeadingCorrection() (jika diaktifkan)
   bluetooth/rc/pathPlanner.begin() (jika diaktifkan)
```

## 10. Perbaikan & Peluang

### 10.1 Ekspansi

- Tambahkan odometri/encoder ke Path Planner
- Tambahkan IMU heading filter (complementary) untuk drift rendah
- Tambahkan PWM speed controller ke servos (saat ini open-loop)
- Tambahkan log error ke SD card untuk debugging lanjut
- Tambahkan Autonomous Navigation (A* execution) saat sudah punya odometri

### 10.2 Tuning Potensial

- Tuned gains per lintasan (mis. berbeda untuk tikungan vs lurus)
- Adaptive I term berdasarkan kecepatan
- Fuzzy threshold untuk noise tinggi
- Multi-sensor fusion (IR + ultrasonik + kamera)

## 11. Referensi & Sumber

- uKit Explore 2 Hardware Manual
- Arduino IDE + uKitExplore2.h library
- Robot Motion Control (PID) dasar
- Adaptive thresholding untuk sensor IR
- EEPROM wear leveling (eepromPutHemat)

## 12. Evolusi Kode

**Catatan pengembangan dari AGENTS.md**:
- Logika belok persimpangan non-blocking dipisah dari helper lama belokKiri90/belokKanan90
- Adaptive threshold dinamis diperkenalkan
- Heading correction diperkenalkan
- Mode pembelajaran/racing ditambahkan
- RC support ditambahkan
- Lap timer ditambahkan
- MEMS IMU drift correction
- Fuzzy PID opsional
- Path Planner opsional (manual)

**Gotchas yang harus dipertahankan**:
1. eepromPutHemat template di header + deklarasi forward sebelum setup()
2. Logika belok persimpangan non-blocking via SUBSTATE_INTER_CREEP / SUBSTATE_TURNING
3. Helper lama belokKiri90/belokKanan90 tetap untuk backward compatibility
4. I term dijepit ±50, reset integral setelah recovery
5. Ultrasonic prioritas tertinggi saat jalan
6. Lost-line timeout 2000ms lalu berhenti (merah)
7. Stuck detector bisa stop robot
8. Sensor thresholds per-sensor dengan clamping 3-18
9. Path planner hanya di sketch root jika diaktifkan
10. Serial output dikontrol oleh ENABLE_SERIAL_OUTPUT/ENABLE_SERIAL_MODE
