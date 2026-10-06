# AGENTS.md

Single-sketch Arduino repo: `LineFollowerFin.ino` is the entire program (PID line follower for uKit Explore 2 robot). No build/test/lint config, no packages, no CI.

## Toolchain
- Target: Arduino (uKit Explore 2 board). Requires external `uKitExplore2.h` library + built-in `EEPROM.h` — neither is vendored here.
- Verify by compiling `LineFollowerFin.ino` in Arduino IDE with the uKitExplore2 library installed. There is no automated test or check script.

## Architecture (all in one file)
- `setup()`: `Initialization()` → `bacaNilaiDariEEPROM()` → `mulaiKalibrasiSensor()` (spins 3s at boot, auto-computes `AMBANG_BATAS`, clamps to 5–14).
- `loop()`: `cekTombol()` → run/stop state machine with 3 branches when running: intersection (both sensors ≥ threshold) → lost-line recovery (both < threshold, spins toward `lastError` side) → normal PID tracer with adaptive base speed.
- Motor mapping: servos 4+2 = left side (dir `1`), servos 1+3 = right side (dir `0`). Speed range 0–150. Keep this polarity when editing drive helpers.
- Persistence: `Kp`, `Kd`, `kecepatanDasarMaks`, `AMBANG_BATAS` stored via `EEPROM.put/get` at byte offsets 4/8/12/16 with magic `123` at addr 0. Note: `Ki` and `kecepatanDasarMin` are NOT persisted.

## Onboard tuning UI (no serial input needed)
- Button 1 short presses adjust Kp (±1.0) or Kd (±0.5) depending on `modeSetelKd`; long-press toggles that mode when stopped, stops robot when running.
- Button 2: ±10 max speed (clamped 60–150); long-press saves EEPROM and starts run.
- RGB LED signals state (white = intersection/start, magenta = lost-line / Kd-tune mode, yellow = calibrating). Preserve these when changing behavior.

## Gotchas
- Sensor convention: `readInfraredDistance(2)` = left, `(1)` = right. Error = `(kiri - kanan)/2`.
- `I` term clamped to ±50; `kecepatanDasar` adapts as `maks - abs(PID)*0.6`, clamped to `[min, maks]`.
- 90° turn helpers block in `while` loops waiting for the line — do not add `delay`-free refactors without keeping that wait.
- Comments/serial prints are in Indonesian — keep that language for consistency.
