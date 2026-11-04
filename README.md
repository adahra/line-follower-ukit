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
- `path_planner.h/.cpp` — path planner (A*) opsional, kini berada di root sketch folder dan dibungkus `#if ENABLE_PATH_PLANNER`.
- `LineFollowerFin.ino` — wiring objek + `setup()` + state machine `loop()`.

## Fitur Utama

### Kontrol & Pergerakan
- **PID**: Error = `(kiri - kanan)/2`. `I` dijepit ±50. Kecepatan dasar adaptif: `kecepatanDasar = kecepatanDasarMaks - |PID| * 0.35` (faktor adaptif), dijepit ke `[kecepatanDasarMin, kecepatanDasarMaks]`.
- **Turbo/Dynamic Speed**: Tersedia profil turbo (`KECEPATAN_TURBO`, `LURUS_THRESHOLD`, `LURUS_TIME_REQUIRED`) untuk kondisi lurus.
- **Servo motor mapping**: Servo 4+2 = sisi kiri (arah `1`), servo 1+3 = sisi kanan (arah `0`). Kecepatan diderate ke **0–150** (deadband 25, slew limit ±14/loop secara default).
- **State machine**: Persimpangan (kedua sensor ≥ ambang), lost-line recovery (kedua sensor < ambang, putar ke arah `lastError`), atau tracer PID normal.

### Mode Operasi
- **NORMAL** (default): mode jalur standar PID.
- **LEARNING**: saat mode ini, event persimpangan dicatat oleh `Mapper` (`recordEvent`) untuk pembelajaran jalur.
- **RACING**: menggunakan prediksi event dari `Mapper` (`checkPrediction`) untuk optimasi (feedback LED menunjukkan arah prediksi).

Mode dapat diatur via antarmuka/serial (lihat `antarmuka`).

### Kalibrasi Manual
- Tekan **kedua tombol bersamaan** saat robot berhenti untuk memulai kalibrasi sensor (berputar ~3 detik, LED kuning).
- Tiap sensor dihitung ambangnya sendiri dari rata-rata max/min, lalu **dijepit ke 3–18** dan disimpan ke EEPROM.
- Tidak ada auto-kalibrasi saat boot secara default (`AUTO_CALIBRATE_ON_BOOT = false`).

### Tuning Tombol Onboard
- **Tombol 1 (pendek)**: `Kp ±1.0` atau `Kd ±0.5` tergantung `modeSetelKd`. **Tombol 1 (lama)**: saat berhenti toggle mode Kp/Kd; saat berjalan menghentikan robot.
- **Tombol 2 (pendek)**: `kecepatanDasarMaks ±10` (dijepit **60–150**). **Tombol 2 (lama)**: simpan ke EEPROM dan mulai jalan.
- **LED RGB**: putih = persimpangan/start, magenta = garis hilang / mode setel Kd, kuning = kalibrasi, hijau berkedip 3× = kalibrasi selesai. Saat berhenti: magenta = mode Kd-tune, kuning = mode LEARNING, hijau = mode RACING, mati = NORMAL.

### Persistence (EEPROM)
- Penanda valid: `123` di alamat `ADDR_EEPROM_CHECK` (0).
- Tersimpan: `Kp` (4), `Ki` (8), `Kd` (12), `kecepatanDasarMaks` (16), `AMBANG_KIRI` (20), `AMBANG_KANAN` (22), peta waypoint (24–266), parameter heading/adaptive (280–297).
- **Tidak disimpan**: `kecepatanDasarMin` (selalu pakai nilai default di `config.h`).
- Penulisan EEPROM dihindari bila nilai tidak berubah (`eepromPutHemat` template di header; butuh forward declaration eksplisit).

### Safety & Fitur Pendukung
- **Ultrasonic stop** (prioritas tertinggi saat berjalan): berhenti bila halangan depan ≤ **5 cm** (`ULTRASONIC_ID`, `JARAK_BERHENTI_CM`).
- **Auto headlight**: eye lamp menyala otomatis saat gelap (< 100 lux, mati > 150 lux), dengan histeresis (`LIGHT_SENSOR_ID`).
- **Clap toggle**: jalan/berhenti via tepuk tangan (`SOUND_SENSOR_ID`, dengan ambang + histeresis + cooldown).
- **Tilt halt** (IMU): berhenti saat miring/terangkat (> `TILT_MAKS_DERAJAT`).
- **Stuck detector**: deteksi macet (IR beku + perubahan akselerasi stabil) dan menghentikan robot.
- **Battery latch**: bila tegangan <= `TEGANGAN_MIN` (5.5V), robot dikunci mati sampai power cycle. Dapat dimatikan dengan `ENABLE_BATTERY_CHECK=false` di `config.h`.
- **Lost-line recovery**: putar cari garis ke arah `lastError`, timeout **2 detik** lalu berhenti (LED merah).
- **Intersection handling**: lewati aman lalu putar 90° kiri/kanan atau lurus berdasarkan pembacaan segar.

### Fitur Opsional (dikendalikan `config.h`)
Semua default **false**:
- **Lap timer** (`ENABLE_LAP_TIMER`): deteksi garis start/finish, hitung waktu lap (`LAP_SF_THRESHOLD`, `LAP_MIN_LAP_MS`).
- **Bluetooth/BLE** (`ENABLE_BLUETOOTH`): kontrol & telemetri via Bluetooth (`BT_BAUD`, `BT_MODULE_TYPE`, `BT_TELEMETRY_MS`).
- **Path planner** (`ENABLE_PATH_PLANNER`): **alat peta/A\* manual** lewat perintah serial (`path ...`), bukan navigasi otonom. Posisi grid robot tidak diketahui tanpa odometri/encoder, dan mengaktifkannya memakai ~94% RAM ATMega2560. File `path_planner.*` ada di root sketch folder dan hanya di-include bila diaktifkan. Lihat [docs/PATH_PLANNER.md](docs/PATH_PLANNER.md) untuk detail algoritma A*; batasan integrasi dijelaskan di [`config.h`](LineFollowerFin/config.h#L211-L218).
- **RC** (`ENABLE_RC`): remote control dua channel (throttle `RC_PIN`, steering `RC_STEER_PIN`), center = berhenti, throttle negatif = mundur. Hanya membaca `pulseIn` saat `MODE_RC` aktif agar tidak memblokir line follower.
- **Fuzzy PID** (`ENABLE_FUZZY_PID`): kontrol PID adaptif berbasis logika fuzzy untuk performa tracking yang lebih halus. Lihat [docs/FUZZY_PID.md](docs/FUZZY_PID.md) untuk dokumentasi lengkap algoritma fuzzy adaptive PID.

Catatan: `TELEMETRI` serial CSV bisa diaktifkan (`ENABLE_TELEMETRI`) untuk tuning; `IMU` digunakan untuk tilt/stuck detection.

## Hardware Konvensi

- **Sensor IR garis**: `readInfraredDistance(2)` = **kiri**, `readInfraredDistance(1)` = **kanan**. Error = `(kiri - kanan)/2`.
- **Motor servo**: 4+2 = sisi kiri (dir `1`), 1+3 = sisi kanan (dir `0`).
- **Kecepatan**: diderate **0–150** (maks efektif via tombol/konfigurasi dibatasi 150). Servo punya deadband; nilai < 25 bisa dipaksa atau tidak efektif.
- **Sensor ID**: `LIGHT_SENSOR_ID`, `ULTRASONIC_ID`, `SOUND_SENSOR_ID` masih bisa perlu disesuaikan dengan wiring.
- **Ambang kalibrasi**: hasil kalibrasi manual dijepit ke **3–18** per sensor.

## EEPROM Layout

| Alamat | Isi | Keterangan |
| --- | --- | --- |
| 3 | `77` | Guard uKit: `Initialization()` menghapus 0–20 bila byte ini bukan 77 |
| 0 | `123` | Magic/check validitas EEPROM |
| 4 | `Kp` (float) | PID proportional |
| 8 | `Ki` (float) | PID integral |
| 12 | `Kd` (float) | PID derivative |
| 16 | `kecepatanDasarMaks` (int) | Batas kecepatan maksimal |
| 20 | `AMBANG_KIRI` (int) | Ambang sensor kiri (hasil kalibrasi/per-tuning) |
| 22 | `AMBANG_KANAN` (int) | Ambang sensor kanan |
| 24–266 | Peta waypoint | Magic `77` + count + 30 × `Waypoint` (8 byte) |
| 280 | `HEADING_GAIN` (float) | Gain koreksi heading |
| 284 | `HEADING_TARGET` (float) | Target heading tersimpan |
| 288 | `AMBANG_KIRI_BASE` (int) | Ambang kiri dasar (adaptive threshold) |
| 290 | `AMBANG_KANAN_BASE` (int) | Ambang kanan dasar (adaptive threshold) |
| 292 | `HEADING_BIAS` (float) | Bias gyro hasil kalibrasi |
| 296 | `88` | Magic data heading |
| 297 | `66` | Magic data adaptive threshold |

`kecepatanDasarMin` **tidak disimpan** (selalu default dari kode).

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
