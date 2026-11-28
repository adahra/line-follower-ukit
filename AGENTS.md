# AGENTS.md

## Ringkasan Proyek & Tech Stack

Robot line follower berbasis PID untuk board **uKit Explore 2** (Arduino Mega2560). Sketch utama `LineFollowerFin/LineFollowerFin.ino` berisi objek, `setup()`, dan state machine `loop()`; logika dipisah ke kelas modular (`sensor_garis`, `penggerak`, `pengaman`, `antarmuka`, `mapping`, `lap_timer`, dst.). Toolchain: Arduino IDE atau `arduino-cli` dengan board `arduino:avr:mega`, library eksternal `uKitExplore2` (tidak vendored), built-in `EEPROM.h`.

## Arsitektur & Struktur Direktori

- `LineFollowerFin/` — sketch Arduino (nama folder harus sama dengan `.ino`)
  - `LineFollowerFin.ino` — objek, `setup()`, state machine `loop()`
  - `config.h` / `config.cpp` — semua nilai tunable (PID, speed, ambang, flag `ENABLE_*`)
  - `status.h` — struct `Status` shared
  - `fwd.h` — forward declarations (wajib untuk template `eepromPutHemat`)
  - `eeprom_layout.h` — layout EEPROM versi struct + `static_assert`
  - `sensor_garis.*` — filter IR, ambang per-sensor, kalibrasi, baca EEPROM
  - `penggerak.*` — komputasi PID, deadband/slew, gerak motor servo, telemetry
  - `pengaman.*` — battery latch, tilt halt, ultrasonic stop, stuck detector, lampu otomatis
  - `antarmuka.*` — tombol onboard, tepuk tangan, parser perintah serial
  - `mapping.*` — Mapper learning/racing untuk prediksi event persimpangan
  - `lap_timer.*` — lap timer opsional (start/finish line)
  - `bluetooth_ctrl.*`, `rc_ctrl.*`, `heading_tracker.*`, `path_planner.*` — opsional
  - `tests/` — `TestLapTimer`, `TestBluetooth`, `TestPathPlanner`
- `docs/` — arsitektur, API, design decisions, ukit_explore
- `sim/` / `simulator/` — simulator Python
- `blockly/` — workspace Blockly
- `.github/workflows/compile.yml` — CI compile sketch utama + 3 test sketch
- `BUILD.md` — panduan compile/upload `arduino-cli`
- `WIRING.md` — panduan rangkaian hardware

## Perintah Build, Run, & Test

```bash
# Compile sketch utama (di dalam LineFollowerFin/)
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin.ino

# Compile test sketch
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin/tests/TestLapTimer
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin/tests/TestBluetooth
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin/tests/TestPathPlanner

# Upload
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega LineFollowerFin.ino

# Serial monitor (baud 115200)
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

Fitur opsional di-toggle via `config.h` (`ENABLE_LAP_TIMER`, `ENABLE_HEADING_CORRECTION`, `ENABLE_PATH_PLANNER`, `ENABLE_FUZZY_PID`, `ENABLE_ADAPTIVE_THRESHOLD`, `ENABLE_BLUETOOTH`, `ENABLE_RC`). Default sebagian besar OFF untuk hemat RAM.

## Konvensi & Aturan Penulisan Kode

- **YAGNI**: tidak menambahkan fitur atau abstraksi yang tidak terpakai.
- **Stdlib first**: gunakan `EEPROM.h`, `Arduino.h`, `Servo.h` sebelum kode custom.
- **Minimal code**: satu bar jika memungkinkan; hanya tambah kode yang memperbaiki bug terverifikasi atau mengaktifkan fitur yang diminta.
- **No boilerplate**: hapus script buangan setelah verifikasi.
- **Verifikasi**: selalu verifikasi perubahan dengan tes atau pemeriksaan spesifik.
- **No magic**: semua parameter di `config.h` atau dokumentasi yang jelas.
- **EEPROM**: gunakan template `eepromPutHemat`; tidak boleh menulis langsung ke EEPROM tanpa deteksi perubahan.
- **Non-blocking**: semua UI dan timing menggunakan `millis()`, bukan `delay()`.
- **Error handling**: semua path kritis harus memiliki pengecekan eksplisit (baterai, tilt, ultrasonic, stuck).
- **Dokumentasi**: setiap kelas/fungsi harus memiliki komentar jelas dalam bahasa Inggris.
- **Komentar Indonesia**: komentar dan serial print dalam bahasa Indonesia untuk konsistensi.
- **Modular**: setiap komponen (sensor, PID, safety, UI) dalam kelas sendiri dengan antarmuka jelas.
- **Forward declarations**: diperlukan untuk template `eepromPutHemat` di header.
- **Performance**: pantau RAM usage; nonaktifkan fitur jika RAM < 7KB (ATMega2560 limit).
- **Safety**: semua fungsi kritis harus memiliki kondisi keluar yang eksplisit.
- **Versioning**: `config.h` sebagai single source of truth untuk parameter tunable.
- **No external dependencies**: hanya library bawaan Arduino dan `uKitExplore2`.
- **Consistency**: ikuti konvensi penamaan dan gaya kode yang ada.

## Gotchas (harus dipertahankan)

- `eepromPutHemat` template di header + forward declarations eksplisit sebelum `setup()` (template merusak auto-prototype Arduino).
- Logika belok persimpangan non-blocking via sub-state machine (`SUBSTATE_INTER_CREEP` + `SUBSTATE_TURNING`). Helper lama `belokKiri90`/`belokKanan90` masih ada untuk backward compatibility.
- I-term dijepit ±50; integral reset setelah keluar recovery.
- Ultrasonik prioritas di atas semua logika drive saat berjalan.
- Lost-line timeout 2000ms lalu berhenti (LED merah); stuck detector dapat menghentikan robot.
- Ambang sensor per-sensor (kiri/kanan) dengan clamping 3–18.
- Path planner files di sketch root dan hanya di-include jika `ENABLE_PATH_PLANNER`.
- Serial output dikontrol oleh `ENABLE_SERIAL_OUTPUT` / `ENABLE_SERIAL_MODE` (Bluetooth memaksa serial on).
- Sensor ID (`LIGHT_SENSOR_ID`, `ULTRASONIC_ID`, `SOUND_SENSOR_ID`) bisa berbeda tergantung wiring — sesuaikan di `config.h`.
- Guard byte `77` di alamat 3 agar `Initialization()` uKit tidak menghapus 0–20.