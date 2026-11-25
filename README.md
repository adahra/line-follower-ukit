# Line Follower UKIT

Robot line follower berbasis PID untuk board uKit Explore 2. Proyek ini modular: logika utama dipisah ke kelas (`sensor_garis`, `penggerak`, `pengaman`, `antarmuka`, `mapping`, `lap_timer`, dsb.), sedangkan `LineFollowerFin/LineFollowerFin.ino` hanya berisi objek, `setup()`, dan state machine `loop()`.

## Kebutuhan

- **Arduino IDE** dengan library `uKitExplore2` terinstal (tidak disertakan di repo ini).
- **Arduino CLI** (lihat [BUILD.md](BUILD.md) untuk compile/upload via CLI).
- Library bawaan `EEPROM.h`.
- Untuk verifikasi: compile `LineFollowerFin/LineFollowerFin.ino` di Arduino IDE. CI GitHub Actions mengompilasi sketch utama + tiga test sketch (lihat [`.github/workflows/compile.yml`](.github/workflows/compile.yml)).

## Overview

Robot mengikuti garis menggunakan kontrol PID dengan beberapa mode operasi, fitur keselamatan, kalibrasi manual, tuning via tombol onboard, serta dukungan fitur opsional yang bisa diaktifkan lewat `config.h`.

## Arsitektur Modular

Struktur file utama di `LineFollowerFin/`:

- `config.h` — semua nilai tunable (PID, speed, ambang, timeout, sensor ID, flag `ENABLE_*`).
- `status.h` — shared `Status` struct (`jalan`, `modeSetelKd`, `recoveryMulai`, dll.).
- `fwd.h` — forward declarations (penting karena `eepromPutHemat` template di header).
- `sensor_garis.*` — filter IR, ambang per-sensor, kalibrasi, baca EEPROM.
- `penggerak.*` — komputasi PID, deadband/slew, helper gerak motor (servo), telemetry.
- `antarmuka.*` — tombol onboard, clap toggle (start/stop), serial command parser.
- `pengaman.*` — battery latch, tilt halt (IMU), ultrasonic stop, stuck detector, auto headlight.
- `mapping.*` — learning/racing mapper untuk prediksi event (persimpangan).
- `lap_timer.*` — lap timer opsional (start/finish line).
- `bluetooth_ctrl.*` — kontrol/telemetri Bluetooth opsional.
- `heading_tracker.*` — IMU heading correction (optional).
- `path_planner.h/.cpp` — path planner (A*) opsional, kini berada di root sketch folder dan dibungkus `#if ENABLE_PATH_PLANNER`.
- `LineFollowerFin.ino` — wiring objek + `setup()` + state machine `loop()`.

## Fitur Utama

### Kontrol & Pergerakan
- **PID**: Error = `(kiri - kanan)/2`. `I` dijepit ±50. Kecepatan dasar adaptif: `kecepatanDasar = kecepatanDasarMaks + bonus turbo - |PID| * 0.5` (faktor adaptif), dijepit ke `[kecepatanDasarMin, SERVO_MAKS_DIPAKAI]`.
- **Turbo/Dynamic Speed**: Tersedia profil turbo (`KECEPATAN_TURBO`, `LURUS_THRESHOLD`, `LURUS_TIME_REQUIRED`) untuk kondisi lurus.
- **Servo motor mapping**: Servo 4+2 = sisi kiri (arah `1`), servo 1+3 = sisi kanan (arah `0`). Kecepatan diderate ke **0–150** (deadband 25, slew limit ±14/loop secara default).
- **State machine**: Persimpangan (kedua sensor ≥ ambang), lost-line recovery (kedua sensor < ambang, putar ke arah `lastError`), atau tracer PID normal.

### Mode Operasi
- **NORMAL** (default): mode jalur standar PID.
- **LEARNING**: saat mode ini, event persimpangan dicatat oleh `Mapper` (`recordEvent`) untuk pembelajaran jalur.
- **RACING**: menggunakan prediksi event dari `Mapper` (`checkPrediction`) untuk optimasi (feedback LED menunjukkan arah prediksi).

Mode dapat diatur via antarmuka/serial (lihat `antarmuka`).

### Kalibrasi Manual
- Tekan **kedua tombol bersamaan** saat robot berhenti untuk memulai kalibrasi sensor (berputar hingga **360°** berbasis gyro IMU, LED kuning). Timeout 8 detik; jika putaran <300° kalibrasi dibatalkan dan ambang lama dipertahankan. Set `CALIB_USE_IMU=false` untuk kembali ke timer 3 detik.
- Lewat serial: `calibrate` atau `calibrate auto` untuk robot berputar sendiri; `calibrate manual` untuk motor diam dan robot diputar 360° dengan tangan (timeout 15 detik).
- Tiap sensor dihitung ambangnya sendiri dari rata-rata max/min, lalu **dijepit ke 3–18** dan disimpan ke EEPROM.
- Tidak ada auto-kalibrasi saat boot secara default (`AUTO_CALIBRATE_ON_BOOT = false`).

### Tuning Tombol Onboard
- **Tombol 1 (pendek)**: `Kp ±1.0` atau `Kd ±0.5` tergantung `modeSetelKd`. **Tombol 1 (lama)**: saat berhenti toggle mode Kp/Kd; saat berjalan menghentikan robot.
- **Tombol 2 (pendek)**: `kecepatanDasarMaks ±10` (dijepit **60–150**). **Tombol 2 (lama)**: simpan ke EEPROM dan mulai jalan.
- **LED RGB**: putih = persimpangan/start, magenta = garis hilang / mode setel Kd, kuning = kalibrasi, hijau berkedip 3× = kalibrasi selesai. Saat berhenti: magenta = mode Kd-tune, kuning = mode LEARNING, hijau = mode RACING, mati = NORMAL.

### Persistence (EEPROM)
- Penanda valid: `123` di alamat `0` (magic/check validitas EEPROM).
- Guard byte `77` di alamat `3` (agar uKit `Initialization()` tidak menghapus 0–20).
- Layout baru: **versi struct `EEPROMData` + `static_assert` compile-time checks** di `eeprom_layout.h`.
- Tersimpan (via `EEPROMData` + helper `eepromWriteIfChanged`):
  - `version` (1), `magic` (123) di alamat 0–1
  - `Kp` (4), `Ki` (8), `Kd` (12) — float
  - `kecepatanDasarMaks` (16) — int16
  - `kecepatanDasarMin` (18) — int16 (hanya default, tidak dipakai runtime)
  - `AMBANG_KIRI` (20), `AMBANG_KANAN` (22) — uint16
  - Peta waypoint: magic `77` (24) + count + 30 × `Waypoint` (8 byte, addr 26–265)
  - `HEADING_GAIN` (280), `HEADING_TARGET` (284) — float
  - `AMBANG_KIRI_BASE` (288), `AMBANG_KANAN_BASE` (290) — uint16
  - `HEADING_BIAS` (292) — float
  - Magic data: heading `0x48` (296), adaptive `66` (297)
- **Tidak disimpan**: `kecepatanDasarMin` (selalu default dari `config.h`).
- Penulisan EEPROM dihindari bila nilai tidak berubah (`eepromWriteIfChanged` di `eeprom_layout.h`; `eepromPutHemat` template di `sensor_garis.h` untuk kompatibilitas lama; butuh forward declaration eksplisit di `fwd.h`).

## EEPROM Layout

| Alamat | Isi | Keterangan |
| --- | --- | --- |
| 0 | `123` | Magic/check validitas EEPROM (`MAGIC_EEPROM_VALID`) |
| 1 | `1` | Format version (`EEPROM_FMT_VERSION`) |
| 3 | `77` | Guard uKit: `Initialization()` menghapus 0–20 bila byte ini bukan 77 |
| 4 | `Kp` (float) | PID proportional |
| 8 | `Ki` (float) | PID integral |
| 12 | `Kd` (float) | PID derivative |
| 16 | `kecepatanDasarMaks` (int16) | Batas kecepatan maksimal |
| 18 | `kecepatanDasarMin` (int16) | Batas kecepatan minimal (default, tidak dipakai runtime) |
| 20 | `AMBANG_KIRI` (uint16) | Ambang sensor kiri (hasil kalibrasi/per-tuning) |
| 22 | `AMBANG_KANAN` (uint16) | Ambang sensor kanan |
| 24 | `77` | Magic map valid (`MAGIC_MAP_VALID`) |
| 26–265 | Waypoint array | 30 × `Waypoint` (8 byte: x,y,t,flags) |
| 280 | `HEADING_GAIN` (float) | Gain koreksi heading |
| 284 | `HEADING_TARGET` (float) | Target heading tersimpan |
| 288 | `AMBANG_KIRI_BASE` (uint16) | Ambang kiri dasar (adaptive threshold) |
| 290 | `AMBANG_KANAN_BASE` (uint16) | Ambang kanan dasar (adaptive threshold) |
| 292 | `HEADING_BIAS` (float) | Bias gyro hasil kalibrasi |
| 296 | `0x48` | Magic data heading (`MAGIC_HEADING_VALID`) |
| 297 | `66` | Magic data adaptive threshold (`MAGIC_ADAPT_VALID`) |

`kecepatanDasarMin` **tidak disimpan** (selalu default dari kode).

## Validasi Input Serial

Semua perintah numerik serial sekarang divalidasi batasnya (bounds checking) sebelum diterapkan. Jika nilai di luar batas, pesan error Indonesia dikirim ke Serial Monitor dan nilai tidak diubah:

| Perintah | Parameter | Batas Valid | Error Message |
| --- | --- | --- | --- |
| `kp` | proportional gain | 0.0 – 50.0 | `ERROR: Kp harus 0.0 - 50.0` |
| `ki` | integral gain | 0.0 – 5.0 | `ERROR: Ki harus 0.0 - 5.0` |
| `kd` | derivative gain | 0.0 – 50.0 | `ERROR: Kd harus 0.0 - 50.0` |
| `speed` | kecepatan maks | 60 – 150 | `ERROR: Speed maks harus 60-150` |
| `speedmin` | kecepatan min | 10 – 150 | `ERROR: Speed min harus 10-150` |
| `ambang` / `ambangk` / `ambangn` | ambang sensor | 1 – 20 | `ERROR: Ambang harus 1-20` (atau kiri/kanan) |
| `lapconfig` threshold | start/finish threshold | 1 – 20 | `ERROR: Lap threshold harus 1-20` |
| `lapconfig` minLap | minimal lap time | 500 – 60000 ms | `ERROR: Min lap harus 500-60000 ms` |
| `htarget` | heading target | -180 – 180° | `ERROR: Heading target -180 s/d 180 derajat` |

Contoh:
```
kp 100      -> ERROR: Kp harus 0.0 - 50.0
kp 7.5      -> Kp = 7.50
speed 200   -> ERROR: Speed maks harus 60-150
speed 100   -> Speed maks = 100
htarget 300 -> ERROR: Heading target -180 s/d 180 derajat
htarget 90  -> Heading target = 90.00
```

## Build / Verify

1. Buka `LineFollowerFin/LineFollowerFin.ino` di Arduino IDE.
2. Pastikan library `uKitExplore2` terinstal.
3. Pilih board uKit Explore 2 yang sesuai.
4. Compile/Upload. Untuk test sketch: `arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin/tests/TestLapTimer` (dan `TestBluetooth`, `TestPathPlanner`).
5. Untuk CLI: lihat [BUILD.md](BUILD.md).

## Gotchas

- `eepromPutHemat` adalah template di header dan bisa merusak auto-prototype generator Arduino — butuh **forward declaration eksplisit** sebelum `setup()` (jangan hapus saat menambah fungsi).
- Helper belok 90° (`belokKiri90`/`belokKanan90`) berjalan di `while` loop menunggu garis — **blocking**. Jangan refactor jadi non-blocking tanpa mempertimbangkan perilaku ini.
- Komentar/serial print sebagian besar berbahasa Indonesia — pertahankan konsistensi.
- Sensor ID (cahaya/ultrasonik/suara) bisa berbeda tergantung wiring — sesuaikan di `config.h`.
- Fitur opsional (`BLUETOOTH`, `PATH_PLANNER`, `LAP_TIMER`) default off karena pertimbangan RAM/kompleksitas; `path_planner` sudah dipindah ke root sketch dan dibungkus `#if`.
- Ultrasonik diprioritaskan di atas logika jalan saat berjalan.
- Saat kembali dari lost-line recovery, integral direset untuk menghindari "hutang" koreksi lama.
