# Dokumentasi Per Modul - LineFollowerFin

Dokumentasi teknis lengkap untuk semua modul inti LineFollowerFin. Digunakan untuk pengembangan, debugging, dan pemeliharaan.

## 1. Ikhtisar

Dokumentasi ini mencakup semua modul inti LineFollowerFin:

- **SensorGaris** - Modul sensor IR yang membaca posisi robot relatif terhadap jalur hitam
- **Penggerak** - Controller PID utama yang mengubah error pelacakan garis menjadi command motor
- **Pengaman** - Sistem deteksi berbahaya, berhenti otomatis, dan kontrol lampu LED
- **Antarmuka** - Kontrol manusia: tombol, tepuk tangan, dan serial command
- **Mapper** - Memory waypoint & racing prediction untuk navigasi balapan
- **LapTimer** - Hitung waktu & statistik lap (opsional)
- **BluetoothCtrl** - Bluetooth control + telemetry
- **RcCtrl** - Remote control mode
- **FuzzyPID** - Adaptive PID gains
- **HeadingTracker** - IMU heading correction
- **PathPlanner** - Manual grid-based path planning (A*)

## 2. Struktur

```
docs/architecture/modules/
├── index.md                           # Index per modul
├── sensor_garis.md                    # SensorGaris - Dokumentasi Teknis
├── penggerak.md                       # Penggerak - Dokumentasi Teknis
├── pengaman.md                        # Pengaman - Dokumentasi Teknis
├── antarmuka.md                       # Antarmuka - Dokumentasi Teknis
├── mapping.md                         # Mapper - Dokumentasi Teknis
├── lap_timer.md                       # LapTimer - Dokumentasi Teknis
├── bluetooth_ctrl.md                  # BluetoothCtrl - Dokumentasi Teknis
├── rc_ctrl.md                         # RcCtrl - Dokumentasi Teknis
├── fuzzy_pid.md                       # FuzzyPID - Dokumentasi Teknis
├── heading_tracker.md                 # HeadingTracker - Dokumentasi Teknis
└── path_planner.md                    # PathPlanner - Dokumentasi Teknis
```

## 3. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/` untuk keputusan desain

## 4. Tipe Data Bersama

### Enum Kontrol
```cpp
enum SubState {
  SUBSTATE_TRACKING = 0,
  SUBSTATE_INTER_CREEP = 1,
  SUBSTATE_TURNING = 2
};

enum ModeOperasi {
  MODE_NORMAL = 0,
  MODE_LEARNING,
  MODE_RACING,
  MODE_RC
};
```

### Enum Sensor
```cpp
enum EventType {
  EVENT_NONE = 0,
  EVENT_INTERSECTION, // Persimpangan
  EVENT_FINISH
};
```

## 5. Kinerja & Sumber Daya

### 5.1 Penggunaan RAM
| Modul | Penggunaan RAM | Keterangan |
|-------|---------------|-------------|
| SensorGaris | ~24 bytes | Filter + adaptive threshold |
| Penggerak | ~50-100 bytes | PID state + gain scheduling |
| Pengaman | ~5 bytes | Latch safety |
| Antarmuka | ~100 bytes | Command parsing |
| Mapper | ~400 bytes | Waypoint array |
| LapTimer | ~320 bytes | Array data lap |
| BluetoothCtrl | ~180 bytes | Buffer telemetry |
| RcCtrl | ~180 bytes | Buffer signal |
| FuzzyPID | ~82 bytes | Rule base + fuzzification |
| HeadingTracker | ~32 bytes | State heading |
| PathPlanner | ~20000 bytes | Grid cells |

### 5.2 Konsumsi Waktu
| Modul | Konsumsi Waktu per Loop | Keterangan |
|-------|--------------------------|-------------|
| SensorGaris | 100-200 microseconds | Filter eksponensial |
| Penggerak | 200-400 microseconds | Loop PID |
| Pengaman | 50-200 microseconds | Throttle safety |
| Antarmuka | 10-50 microseconds | Parsing command |
| Mapper | 10-50 microseconds | Prediction |
| LapTimer | 5-10 microseconds | Update lap timer |
| BluetoothCtrl | 10-50 microseconds | Update Bluetooth |
| RcCtrl | 10-50 microseconds | Update RC |
| FuzzyPID | 100-150 microseconds | Compute fuzzy |
| HeadingTracker | 15-25 microseconds | Update heading |
| PathPlanner | 0-100 microseconds | Update map |

## 6. Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/api/` untuk dokumentasi API lengkap
- Lihat `docs/design-decisions/` untuk keputusan desain