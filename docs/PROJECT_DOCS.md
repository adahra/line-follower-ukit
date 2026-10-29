# Line Follower UKIT - Documentation

## Project Overview
**Line Follower UKIT** is a PID-based line-following robot built on the uKit Explore 2 board. The entire sketch is modular, with logic split across multiple `.h`/`.cpp` class files, all wired together via references in the main `.ino` file.

The robot follows a black line on a white surface using IR sensors, with PID control for smooth tracking, and includes numerous safety features, auto-tuning capabilities, and an onboard tuning interface that requires no serial monitor.

---

## Table of Contents
1. [Tech Stack](#tech-stack)
2. [Hardware Platform](#hardware-platform)
3. [Software Architecture](#software-architecture)
4. [Class Reference](#class-reference)
5. [Configuration](#configuration)
6. [Operation Modes](#operation-modes)
7. [Onboard Tuning UI](#onboard-tuning-ui)
8. [EEPROM Persistence](#eeprom-persistence)
9. [Safety Features](#safety-features)
10. [Known Gotchas & Constraints](#gotchas--constraints)
11. [Building & Verifying](#building--verifying)

---

## Tech Stack

| Layer | Technology |
|-------|-----------|
| **Framework** | Arduino IDE + Arduino core |
| **Language** | C++ |
| **Target Board** | uKit Explore 2 |
| **External Libraries** (must be installed in Arduino IDE) | `uKitExplore2.h`, `ClickButton.h`, `uKitMotor.h`, `uKitServo.h`, `uKitSensor.h`, `uKitId.h`, `onBoardHW.h`, `TransforRobot.h`, `VisionDiscrim.h`, `KalmanMPU6050.h` |
| **Built-in Libraries** | `EEPROM.h`, `Arduino.h` |
| **Build System** | Manual Arduino compile (no CI/lint/test) |

---

## Hardware Platform

- **Microcontroller**: uKit Explore 2 board
- **Sensors**:
  - IR line sensors (2 units): `readInfraredDistance(2)` = Left, `readInfraredDistance(1)` = Right
  - Ultrasonic distance sensor (front): `readUltrasonicDistance(id)`
  - Light sensor (for auto headlight): `readLightValue(id)` (0-4000 lux)
  - Sound sensor (for clap toggle): `readSoundValue(id)`
  - IMU (MPU6050): for tilt detection and stuck detection
- **Actuators**:
  - 4x continuous rotation servos (mapped as motors):
    - Servos 4+2 = Left side (direction `1`)
    - Servos 1+3 = Right side (direction `0`)
  - RGB LED (onboard)
  - Eye lamps (2 units: kiri/kanan)
- **Input**:
  - 2x onboard buttons (Button 1, Button 2)
- **Power**: Battery voltage monitoring

**Sensor Convention**:
- `readInfraredDistance(2)` = Left sensor
- `readInfraredDistance(1)` = Right sensor
- Error calculation: `(kiri - kanan) / 2`

---

## Software Architecture

The project uses a modular class-based architecture with **one `.h`/`.cpp` pair per functional block**, all wired via references in `LineFollowerFin.ino`.

### Directory Structure (`LineFollowerFin/`)
```
├── config.h/.cpp           — All tunable values (PID, speeds, thresholds, timeouts, sensor IDs)
├── status.h                — Shared Status struct (jalan, modeSetelKd, recoveryMulai, modeOperasi)
├── sensor_garis.h/.cpp     — IR filter, per-sensor thresholds, calibration, EEPROM (incl. eepromPutHemat template)
├── penggerak.h/.cpp        — PID compute, deadband/slew, all motor helpers, telemetry
├── antarmuka.h/.cpp        — Onboard buttons + clap toggle (start/stop sequences)
├── pengaman.h/.cpp         — Battery latch, tilt halt, ultrasonic stop, stuck detector, auto headlight
├── mapping.h               — Event recording/prediction for racing mode (waypoints)
├── fwd.h                   — Forward declarations to avoid circular includes
├── ukit_wrapper.h          — Macros wrapping uKit library objects (include in .cpp files)
├── ukit_extern.h           — Extern declarations for uKit globals (include in .cpp files)
├── ukit_macros.h           — Only macros from ucode.h, no global definitions
├── mapping.h               — Racing mode waypoint mapping
└── LineFollowerFin.ino     — Main sketch: objects, setup(), loop() state machine
```

### Key Design Notes
- **All classes use reference semantics** — objects are declared in `.ino` and passed by reference to constructors/ methods.
- **Forward declarations in `fwd.h`** — mandatory because the `eepromPutHemat` template breaks Arduino's auto-prototype generator. Explicit forward declarations must exist before `setup()`.
- **No auto-calibration at boot** — manual calibration via pressing both buttons together while stopped (spins 3s, auto-computes per-sensor thresholds `AMBANG_KIRI`/`AMBANG_KANAN`, each clamped to 5–14).
- **All comments/serial prints in Indonesian** — preserve this language consistency when adding/editing code.

---

## Class Reference

### `Status` (status.h)
```cpp
struct Status {
  bool jalan = false;          // Robot running flag
  bool modeSetelKd = false;   // Kd tuning mode active
  unsigned long recoveryMulai = 0;  // 0 = not in recovery mode
  ModeOperasi modeOperasi = MODE_NORMAL;  // MODE_NORMAL, MODE_LEARNING, MODE_RACING
};
```

### `SensorGaris` (sensor_garis.h)
- `bacaKiri()` — Read left sensor (ID 2), already filtered
- `bacaKanan()` — Read right sensor (ID 1), already filtered
- `kalibrasi(Penggerak &gerak)` — Manual calibration: spin 3s, compute per-sensor thresholds, clamped to 5–14
- `simpan()` — Save Kp/Kd/speed/ambang to EEPROM (using `eepromPutHemat`)
- `muat()` — Load from EEPROM (validated by magic number `123` at addr 0)

### `Penggerak` (penggerak.h)
- `hitung(int bacaKiri, int bacaKanan, bool paksa = false)` — Compute PID + apply to motors. Returns `false` if PID interval not yet elapsed.
- `resetIntegralSetelahRecovery(float errorBaru)` — Clear integral term when coming back from lost-line recovery.
- `putarCariGaris(float lastErr)` — Spin in place toward `lastError` side during lost-line recovery.
- `belokKiri90(int ambang)` / `belokKanan90(int ambang)` — 90° turn helpers (blocking `while` loops waiting for line).
- `errorTerakhir()` — Return last error value.
- `telemetri(int bacaKiri, int bacaKanan)` — Dump CSV telemetry data.

### `Pengaman` (pengaman.h)
- `bateraiKritis()` — Return true if battery voltage ≤ `TEGANGAN_MIN` (5.5V), locking all functions until power cycle. Gated by `ENABLE_BATERAI_KRITIS`; emits orange LED + serial when `TEGANGAN_PERINGATAN` exceeded.
- `miring()` — Return true if IMU tilt > `TILT_MAKS_DERAJAT` (45°).
- `halangan()` — Return true if ultrasonic distance ≤ `JARAK_BERHENTI_CM` (5 cm).
- `macet(int bacaKiri, int bacaKanan)` — Return true if robot is stuck (IR readings stable + accelerometer magnitude stable for `STUCK_DIAM_MS` = 2000ms).
- `lampuOtomatis()` — Auto headlight: on when light < 100 lux, off when > 150 lux (hysteresis).

### `Antarmuka` (antarmuka.h)
- `cekTombol()` — Non-blocking button checking with 300ms cooldown.
- `cekTepuk()` — Clap toggle: 1x tap = jalan/stop toggle.
- `cekSerial()` — Serial command parser for Kp/Ki/Kd/speed/ambang/calibrate/save/help.

### `Mapper` (mapping.h)
- `recordEvent(EventType type, ActionType action)` — Record waypoint during learning mode.
- `checkPrediction(unsigned long currentTime, EventType &outType, ActionType &outAction)` — Predict next event in racing mode.

---

## Configuration (`config.h`)

All tunable values are in one place. **Tune here** — do not scatter constants through the code.

### PID Parameters
- `extern float Kp` — Proportional gain
- `extern float Ki` — Integral gain (NOT persisted to EEPROM)
- `extern float Kd` — Derivative gain
- `extern int kecepatanDasarMaks` — Max speed baseline (clamped 60–150 via button)
- `extern int kecepatanDasarMin` — Min speed (NOT persisted to EEPROM)
- `extern int kecepatanDasar` — Dynamic speed, adapted by PID

### PID Constraints
- `const float FAKTOR_REM_ADAPTIF = 0.6` — Aggressiveness of speed reduction when veering
- `const float BATAS_INTEGRAL = 50` — Anti-windup clamp for I term (±50)
- `const int SPEED_TOMBOL_MIN = 60` — Min speed via button
- `const int SPEED_TOMBOL_MAKS = 150` — Max speed (also derated servo limit)

### Dynamic Speed Profile (Turbo Mode)
- `const int KECEPATAN_TURBO = 150` — Max speed on straightaways
- `const float LURUS_THRESHOLD = 2.0` — If `abs(error) < 2.0`, considered straight
- `const unsigned long LURUS_TIME_REQUIRED = 500` — Must be straight for 500ms before Turbo
- `const int TURBO_RAMP_SPEED = 2` — Speed ramp per update to avoid slipping

### Servo Limits
- `const int SERVO_MIN_EFEKTIF = 40` — Speeds 1–40 forced to 40 or 0 (deadband compensation)
- `const int SERVO_MAKS_DIPAKAI = 150` — Derated from 255; do not increase without track testing
- `const int SLEW_MAKS_PER_LOOP = 10` — Max speed change per loop anti-slip

### Sensor IDs (confirm against wiring)
- `const int LIGHT_SENSOR_ID = 1` — ID terverifikasi sesuai wiring sensor cahaya
- `const int EYELIGHT_KIRI = 1`, `EYELIGHT_KANAN = 2`
- `const int AMBANG_GELAP_NYALA = 100` lux — below this, headlight on
- `const int AMBANG_GELAP_MATI = 150` lux — above this, headlight off (hysteresis)
- `const int ULTRASONIC_ID = 1` — ID terverifikasi sesuai wiring ultrasonic depan
- `const int JARAK_BERHENTI_CM = 5` — stop if obstacle ≤ 5 cm
- `const int SOUND_SENSOR_ID = 1`
- `const int TEPUK_AMBANG = 500`, `TEPUK_LEPAS = 350` — clap detection thresholds

### EEPROM Addresses
| Address | Value |
|---------|-------|
| 0 | Magic number `123` (validates EEPROM data) |
| 4 | `Kp` (float, 4 bytes) |
| 8 | `Ki` (float, 4 bytes — but NOT persisted) |
| 12 | `Kd` (float, 4 bytes) |
| 16 | `kecepatanDasarMaks` (int, 2 bytes) |
| 18 | `AMBANG_KIRI` (int, 2 bytes) |
| 20 | (unused/reserved) |
| 22 | `AMBANG_KANAN` (int, 2 bytes — newer format) |

### Boot Options
- `const bool AUTO_CALIBRATE_ON_BOOT = false` — If true, auto-calibrate on invalid EEPROM at boot
- `const bool ENABLE_TELEMETRI = false` — If true, send CSV telemetry via serial (uses RAM + Flash)

---

## Operation Modes

The robot has 3 primary modes, plus sub-modes:

### MODE_NORMAL (default)
- PID line following with adaptive speed
- Onboard tuning via buttons
- Auto headlight, ultrasonic stop, tilt halt

### MODE_LEARNING
- Fitur tambahan, bukan mode default.
- Records waypoints via `mapper.recordEvent()` at intersections.
- Menyimpan maksimal 50 waypoint di RAM, tiap waypoint berisi tipe event, aksi, dan offset waktu sejak event sebelumnya.
- RGB: Kuning (yellow).
- Data dipakai sebagai dasar prediksi Racing; penyimpanan waypoint ke EEPROM belum tersedia walau command `save` mencetak waypoint.

### MODE_RACING
- Fitur tambahan, bukan mode default.
- Uses `mapper.checkPrediction()` to anticipate upcoming intersections/actions berdasarkan waktu relatif sesi learning.
- Prediction hanya aktif jika waypoint tersedia di instance `Mapper` yang sama.
- Toleransi prediction: 100 ms.
- RGB: Hijau (green) saat berhenti; saat berjalan prediction mengubah LED menjadi putih/hijau/merah/biru sesuai aksi.
- Prediction saat ini hanya memberi feedback LED dan belum mengubah keputusan belok atau speed secara langsung.

### Sub-states within `loop()` (when `status.jalan == true`):
1. **Intersection detection** (both sensors ≥ own threshold):
   - LED Putih (white)
   - Drive through at safe speed (`kecepatanDasarMin`, 150)
   - Then decide: straight, left 90°, or right 90° based on which sensor sees more reflection
   - Record event if in learning mode

2. **Lost-line recovery** (both sensors < own threshold):
   - LED Magenta (magenta)
   - Spin toward `lastError` side via `gerak.putarCariGaris()`
   - Timeout after 2s (`RECOVERY_TIMEOUT_MS`); if timeout, stop robot and wait for operator
   - RGB: Merah/Magenta

3. **Normal PID tracing** (one or both sensors near threshold):
   - Adaptive speed: `kecepatanDasar = maks - abs(PID) × 0.6`, clamped to `[min, maks]`
   - I term clamped to ±50
   - LED based on mode (see below)

### Mode-Independent States
- **Stopped** (`status.jalan == false`):
  - `gerak.saatBerhenti()` — halt motors, clear slew memory
  - Button 1 long-press toggles `modeSetelKd` (Kd tuning mode)
  - Button 2 long-press saves EEPROM and starts run
  - RGB signals: magenta (Kd tune), kuning (learning), hijau (racing), putih (normal start), mati (idle)

---

## Auto-Tuning PID (Fitur Tambahan)

Auto-tuning bukan bagian dari loop PID normal. Sequence dipanggil saat robot berhenti melalui command `tune` atau long press Button 1 + Button 2.

Urutan aktual di `Antarmuka::tuningSequence()`:

1. Kalibrasi sensor.
2. Grid search 16 kombinasi `Kp` `{15, 20, 25, 30}` dan `Kd` `{0.5, 1.0, 1.5, 2.0}`.
3. Set `Ki = 0.01` selama pengujian.
4. Uji tiap kombinasi selama 5 detik.
5. Skor memakai rata-rata `abs(error)` dan penalti oscillation.
6. Terapkan `Kp`/`Kd` terbaik.
7. Simpan parameter dan ambang ke EEPROM.

Catatan implementasi: fungsi auto-tuning menjalankan loop blocking. Proteksi utama di `loop()` tidak dipanggil selama pengujian, dan deteksi oscillation aktual perlu audit karena `prevError` diperbarui sebelum pemeriksaan perubahan tanda. Gunakan fitur ini hanya pada lintasan aman.

## Onboard Tuning UI (No Serial Monitor Needed)

All PID and speed tuning can be done using the two onboard buttons **without connecting to a computer**.

### Button 1 (Short Press)
- **When stopped**: Adjusts `Kp` (±1.0) or `Kd` (±0.5) depending on `modeSetelKd`
  - If `modeSetelKd == false`: Adjusts Kp
  - If `modeSetelKd == true`: Adjusts Kd
- **Long-press** (while stopped): Toggles `modeSetelKd` (switch between Kp/Kd tuning)
- **Long-press** (while running): Emergency stop

### Button 2 (Short Press)
- **When stopped**: Increases max speed by 10 (clamped 60–150)
- **When running**: Has no effect (only long-press matters)
- **Long-press** (while stopped): Saves current config to EEPROM and starts the robot

### LED Color Signals
| State | Color | Meaning |
|-------|-------|---------|
| White | Putih | Intersection / start |
| Magenta | Magenta | Lost-line / Kd-tune mode |
| Yellow | Kuning | Calibrating |
| Green | Hijau | Racing mode |
| Yellow | Kuning | Learning mode |
| Cyan/Magenta | 255,0,255 | During lost-line recovery |
| Red | Merah | Stuck / recovery timeout |
| Off | Mati | Idle / not running |

### Clap Toggle
- 1 tap on sound sensor = toggle jalan/berhenti
- Hysteresis + cooldown prevent double-registering one clap
- Configurable via `TEPUK_AMBANG` (default 500) and `TEPUK_LEPAS` (default 350)

---

## EEPROM Persistence

Configuration is persisted to EEPROM at boot and on explicit save:

### What IS Persisted
- `Kp` (float, addr 4)
- `Kd` (float, addr 12)
- `kecepatanDasarMaks` (int, addr 16)
- `AMBANG_KIRI` (int, addr 18 — was single threshold; now separate left/right)
- `AMBANG_KANAN` (int, addr 22 — new format, used if EEPROM magic exists)
- Magic number `123` at addr 0 (validates that EEPROM data is valid)

### What is NOT Persisted
- `Ki` — always uses default/code value
- `kecepatanDasarMin` — always uses default/code value

### Write Strategy: `eepromPutHemat` Template
Defined in `sensor_garis.h`:
```cpp
template <typename T> inline void eepromPutHemat(int addr, const T &nilai) {
  T lama;
  EEPROM.get(addr, lama);
  if (memcmp(&lama, &nilai, sizeof(T)) != 0) {
    EEPROM.put(addr, nilai);
  }
}
```
- **Writes only when the byte value actually changes**
- EEPROM endurance: ~100,000 cycles; manual calibration + save at startup is safe
- Skipped when unchanged (no unnecessary writes)

### Boot EEPROM Logic (in `.ino` setup)
```cpp
bool eepromValid = (EEPROM.read(ADDR_EEPROM_CHECK) == 123);
if (!eepromValid && AUTO_CALIBRATE_ON_BOOT) {
  // Auto-calibrate
  sensor.kalibrasi(gerak);
} else {
  // Load from EEPROM and print values
  sensor.muat();
  // Print battery, ready message
}
```

---

## Safety Features

| Feature | Trigger | Action |
|---------|---------|--------|
| **Battery Critical** | `tegangan() ≤ TEGANGAN_MIN` (5.5V) | Lock all functions; robot stays stopped until power cycle (restart) |
| **Battery Warning** | `TEGANGAN_MIN < tegangan() ≤ TEGANGAN_PERINGATAN` (5.8V) | Orange LED + serial notice; continues running |
| **Tilt/Miring** | IMU tilt > 45° | Motors disabled until robot is flat again |
| **Front Obstacle** | Ultrasonic ≤ 5 cm | **Priority over ALL drive logic** — robot stops immediately |
| **Stuck Detector** | IR stable + accel magnitude stable for 2s | Robot stops; requires operator intervention |
| **Auto Headlight** | Light < 100 lux → ON; Light > 150 lux → OFF (hysteresis) | Non-blocking, throttled at 200ms intervals |
| **Lost-Line Timeout** | Both IR < threshold for > 2s | Stop robot, turn LED red, wait for operator |
| **Cooldown Buttons** | 300ms debounce | Prevents rapid button presses from messing up state |

**Priority Order** (highest first):
1. Battery critical → stop everything
2. Tilt → disable motors
3. Front ultrasonic ≤ 5 cm → stop everything
4. Lost-line recovery (with timeout)
5. Normal PID tracing

---

## Known Gotchas & Constraints

### Must-Preserve Behaviors
1. **90° turn helpers block in `while` loops** — do not add `delay`-free refactors without keeping the wait-for-line behavior
2. **Explicit forward declarations before `setup()` are mandatory** — the `eepromPutHemat` template breaks Arduino's auto-prototype generator
3. **Sensor convention**: `readInfraredDistance(2)` = left, `(1)` = right. Error = `(kiri - kanan)/2`
4. **I term clamped to ±50** — do not remove the anti-windup clamp
5. **Speed derated to 0–150** — docs allow 0–255, but 150 is the intentional derate for stability
6. **Deadband compensation at 25** — speeds 1–24 forced to 25 or 0
7. **Slew limit ±10/loop** — max speed change per loop anti-slip
8. **Comments/serial prints in Indonesian** — keep that language for consistency

### Edge Cases
- **Sensor IDs are fixed by verified wiring**: `LIGHT_SENSOR_ID = 1`, `ULTRASONIC_ID = 1`.
- **90° turns may not be perfectly accurate** — depends on surface, friction, sensor thresholds
- **EEPROM write skipping** means if you change a value to the same value, it won't be written (useful, but can be confusing during development)
- **`Ki` and `kecepatanDasarMin` are NOT persisted** — they always revert to code defaults on restart
- **Auto-calibration at boot** only happens if `AUTO_CALIBRATE_ON_BOOT = true` AND EEPROM magic is invalid
- **Clap toggle is finicky** — ambient noise can trigger false positives; tune `TEPUK_AMBANG` and `TEPUK_LEPAS` for your environment

### Development Tips
- Use Button 1 + Button 2 long-press to save EEPROM and start running after any config change
- Serial monitor still available for debugging: `cekSerial()` parses commands: `Kp`, `Kd`, `speed`, `ambang`, `calibrate`, `save`, `help`
- Telemetry can be enabled via `ENABLE_TELEMETRI = true` in `config.h` (CSV: `t,err,pid,dasar,kiri,kanan,senK,senN`)
- RGB LED signals are your best friend for debugging without serial

---

## Building & Verifying

### Prerequisites
1. Arduino IDE installed
2. `uKitExplore2` library installed in Arduino IDE (not vendored in this repo)
3. uKit Explore 2 board connected via USB

### Verify Compile
```bash
# Navigate to the sketch directory
cd /home/tovi/Dokumen/line-follower-ukit/LineFollowerFin

# Open Arduino IDE and:
# File → Open → LineFollowerFin/LineFollowerFin.ino
# Verify (✓) or Sketch → Verify/Compile

# Or via command line if arduino-cli is available:
arduino-cli compile -b arduino:avr:uno LineFollowerFin/  # adjust board as needed
```

### No Automated Tests
- There is no automated test or check script
- Manual verification required: upload to board, observe behavior
- Use serial monitor (`Serial Monitor` at 9600 baud) for debugging output

### Typical Workflow
1. Upload sketch to board
2. Observe RGB LED and listen for serial output
3. If first run or EEPROM invalid: press **both buttons together** while stopped → robot spins 3s (calibration), then saves thresholds to EEPROM
4. Press **Button 2 long-press** to save EEPROM and start running
5. Use **Button 1** to adjust Kp/Kd, **Button 2** to adjust speed
6. Observe behavior: line following, intersection handling, lost-line recovery, auto headlight

---

## Project Files Summary

| File | Purpose |
|------|---------|
| `LineFollowerFin.ino` | Main sketch: objects, `setup()`, `loop()` state machine |
| `config.h` | All tunable values — the primary tuning file |
| `status.h` | Shared status struct, passed by reference between modules |
| `sensor_garis.h/.cpp` | IR sensors, filtering, calibration, EEPROM (`eepromPutHemat` template) |
| `penggerak.h/.cpp` | PID computation, motor control, deadband/slew, telemetry |
| `antarmuka.h/.cpp` | Onboard buttons, clap toggle, serial commands |
| `pengaman.h/.cpp` | Safety: battery, tilt, ultrasonic, stuck detection, auto headlight |
| `mapping.h` | Racing mode waypoint recording/prediction |
| `fwd.h` | Forward declarations — mandatory for template compatibility |
| `ukit_wrapper.h` | Macros wrapping uKit library objects (include in .cpp files) |
| `ukit_extern.h` | Extern declarations for uKit globals (include in .cpp files) |
| `ukit_macros.h` | Only macros from ucode.h, no global definitions (include in .cpp files) |

---
*Documentation generated from codebase analysis. Comments and serial prints follow Indonesian language convention.*