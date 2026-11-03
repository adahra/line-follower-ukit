# AGENTS.md

## Structure
- Sketch folder: `LineFollowerFin/` (name must match). Only `objects`, `setup()`, `loop()` in `LineFollowerFin.ino`; logic split into classes via refs.
- Core modules: `config.h/.cpp`, `status.h`, `fwd.h`, `sensor_garis.*`, `penggerak.*`, `antarmuka.*`, `pengaman.*`, `mapping.*`, `lap_timer.*`, `bluetooth_ctrl.*`, `rc_ctrl.*`, `path_planner.*` (optional), `fuzzy_pid.h` (optional).
- `ucode.h` included only in `.ino`; `.cpp` files use `ukit_wrapper.h`/`ukit_extern.h`/`ukit_macros.h`.
- Forward declarations in `fwd.h` are mandatory (template `eepromPutHemat` breaks auto-prototype). Keep `eepromPutHemat` in `sensor_garis.h`.

## Toolchain
- Target: uKit Explore 2 (Arduino Mega2560). Needs external `uKitExplore2.h` + built-ins (`EEPROM.h`, `Arduino.h`). Not vendored.
- Compile: `LineFollowerFin/LineFollowerFin.ino` in Arduino IDE or via arduino-cli (see `BUILD.md`). Tests exist in `LineFollowerFin/tests/` (`TestLapTimer`, `TestBluetooth`, `TestPathPlanner`). CI compiles main + tests. No unit test runner.
- Optional features toggled via `config.h` `ENABLE_*` (Fuzzy PID, Path Planner, Bluetooth, RC, Lap Timer). Default mostly false to save RAM.

## Hardware/Conventions
- Sensor IR: `readInfraredDistance(2)` = kiri, `readInfraredDistance(1)` = kanan. Error = `(kiri - kanan)/2`.
- Motors: servos 4+2 = kiri (dir `1`), servos 1+3 = kanan (dir `0`). Derated to 0–150, deadband ~25, slew ±14/loop by default.
- Sensor IDs (`LIGHT_SENSOR_ID`, `ULTRASONIC_ID`, `SOUND_SENSOR_ID`) may differ by wiring.
- Comments/serial prints mostly Indonesian — keep consistent.

## Flow & State Machine (`loop()`)
Order/priority (from code): protocol() if not running; check battery critical → return; check tilt → return; `ui.cekTombol()`, `ui.cekTepuk()`, `ui.cekSerial()` (if enabled), `aman.lampuOtomatis()`; update optional modules (BT/RC/PathPlanner); if `status.jalan`:
1. Ultrasonic stop ≤5 cm has highest priority → return.
2. RACING prediction via `Mapper` if enabled (LED feedback). RC mode takes over completely if enabled.
3. Intersection (both ≥ threshold): white LED, `majuManual(min,150)`, re-read sensors, decide straight/left/right by comparison, call `belokKiri90`/`belokKanan90` or `majuTerus()`, record to `Mapper` if LEARNING.
4. Lost-line (both < threshold): start recovery timer if first time; if > `RECOVERY_TIMEOUT_MS` (2000ms) stop + red LED; else spin toward `lastError` via `putarCariGaris()`.
5. Normal: check stuck (`aman.macet`), reset integral on recovery exit (`resetIntegralSetelahRecovery`), `gerak.hitung()` uses fixed interval `PID_INTERVAL_MS` (10ms); if not ready call `terapkanTerakhir()` and return; update lap timer if enabled.

## Tuning & EEPROM
- Tuning via buttons (no serial needed): Btn1 short adjusts Kp/Kd depending on `modeSetelKd` (toggle long-press while stopped), long-press while running stops. Btn2 short adjusts max speed ±10 (60–150), long-press saves EEPROM + starts. RGB shows states.
- Persisted: magic `123` at 0, `Kp`(4), `Kd`(12), `kecepatanDasarMaks`(16), `AMBANG_KIRI`(18), `AMBANG_KANAN`(22). Not persisted: `Ki`, `kecepatanDasarMin`. Writes via `eepromPutHemat` (only if changed). Manual calibration: both buttons together while stopped → spin ~3s, compute per-sensor thresholds clamped to 5–14.
- `setup()`: `Initialization()` → `IMU::init()`; if not in protocol run state load EEPROM + mapper, auto-calibrate only if invalid and `AUTO_CALIBRATE_ON_BOOT` true.

## Gotchas (must preserve)
- `eepromPutHemat` template in header + explicit forward decls before `setup()` required.
- 90° turn helpers (`belokKiri90`/`belokKanan90`) block in `while` waiting for line.
- I term clamped ±50; integral reset after recovery. Adaptive speed uses factor in code/config.
- Ultrasonic priority over all drive logic when running.
- Lost-line timeout 2000ms then stop (red). Stuck detector can stop robot.
- Sensor thresholds per-sensor (left/right) with clamping 5–14.
- Path planner files live in sketch root and are only included if `ENABLE_PATH_PLANNER`.
- Serial output controlled by `ENABLE_SERIAL_OUTPUT`/`ENABLE_SERIAL_MODE` (Bluetooth forces serial on).

## Quick Commands
- Verify: open `LineFollowerFin/LineFollowerFin.ino` in Arduino IDE with uKitExplore2 installed.
- CLI: see `BUILD.md` (arduino-cli examples for `arduino:avr:mega`).
- CI: `.github/workflows/compile.yml` compiles main + all three tests.
