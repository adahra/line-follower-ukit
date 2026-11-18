# API Dokumentasi - LineFollowerFin

Dokumentasi API lengkap untuk semua modul LineFollowerFin. Digunakan untuk pengembangan, debugging, dan integrasi.

## Struktur

```
docs/api/
├── index.md                           # Index API
├── sensor_garis.md                    # SensorGaris API
├── penggerak.md                       # Penggerak API
├── pengaman.md                        # Pengaman API
├── antarmuka.md                       # Antarmuka API
├── mapping.md                         # Mapper API
├── lap_timer.md                       # LapTimer API
├── bluetooth_ctrl.md                  # BluetoothCtrl API
├── rc_ctrl.md                         # RcCtrl API
├── fuzzy_pid.md                       # FuzzyPID API
├── heading_tracker.md                 # HeadingTracker API
├── path_planner.md                    # PathPlanner API
└── common.md                          # Tipe data & definisi bersama
```

## Tipe Data Bersama

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

### Enum Bluetooth
```cpp
enum BTModuleType {
  BT_NONE = 0,
  BT_HM10,
  BT_UKIT_BUILTIN
};
```

### Enum RC
```cpp
enum RCSource {
  RC_NONE = 0,
  RC_PWM,
  RC_SERIAL
};
```

### Enum PathPlanner
```cpp
enum CellType {
  CELL_UNKNOWN = 0,
  CELL_EMPTY = 1,
  CELL_LINE = 2,
  CELL_OBSTACLE = 3,
  CELL_START = 4,
  CELL_VISITED = 5
};

enum Direction {
  DIR_N = 0, DIR_NE = 1, DIR_E = 2,
  DIR_SE = 3, DIR_S = 4, DIR_SW = 5,
  DIR_W = 6, DIR_NW = 7
};
```

## Referensi Silang

- Lihat `docs/architecture/LineFollowerFin-architecture.md` untuk arsitektur sistem
- Lihat `docs/architecture/modules/` untuk dokumentasi per modul
- Lihat `docs/design-decisions/` untuk keputusan desain
