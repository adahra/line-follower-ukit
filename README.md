# Line Follower UKIT

Robot line follower berbasis PID untuk board **uKit Explore 2** (ATMega2560). Proyek ini modular: logika utama dipisah ke kelas (`sensor_garis`, `penggerak`, `pengaman`, `antarmuka`, `mapping`, `lap_timer`, dst.), sedangkan `LineFollowerFin/LineFollowerFin.ino` hanya berisi objek, `setup()`, dan state machine `loop()`.

## Fitur Utama

### Kontrol & Pergerakan
- **PID**: Error = `(kiri - kanan)/2`. I-term dijepit ±50. Kecepatan dasar adaptif dengan faktor pengereman adaptif, dijepit ke rentang `[kecepatanDasarMin, SERVO_MAKS_DIPAKAI]`.
- **Profil turbo**: Bonus kecepatan (`KECEPATAN_TURBO`) aktif saat lintasan lurus dan error kecil, dengan ramp perlahan agar tidak slip.
- **Mapping servo**: Servo 4+2 = sisi kiri (arah `1`), servo 1+3 = sisi kanan (arah `0`). Kecepatan diderate ke **0–150** dengan deadband 25 dan slew limit ±12/loop.
- **State machine non-blocking**: Persimpangan (keduanya ≥ ambang), lost-line recovery (keduanya < ambang, putar ke arah `lastError`), atau tracer PID normal. Semua timing menggunakan `millis()`, tidak ada `delay()`.

### Mode Operasi
- **NORMAL** (default): jalur standar PID.
- **LEARNING**: event persimpangan dicatat oleh `Mapper` (`recordEvent`) untuk pembelajaran jalur, persistensi ke EEPROM.
- **RACING**: prediksi event dari `Mapper` (`checkPrediction`) untuk optimasi persimpangan ambigu, ditunjukkan via LED.
- **RC** (opsional): kontrol remote melalui PWM/PPM atau Serial, menggantikan semua logika tracer saat aktif.
- **Bluetooth** (opsional): kontrol dan telemetri melalui modul BT (HM-10 / uKit built-in).

### Kalibrasi Manual
- Tekan **kedua tombol bersamaan** saat robot berhenti → kalibrasi sensor (putar hingga 360° berbasis gyro IMU, LED kuning). Timeout 8 detik; jika putaran <300° kalibrasi dibatalkan dan ambang lama dipertahankan. Set `CALIB_USE_IMU=false` untuk fallback timer 3 detik.
- Lewat serial: `calibrate` / `calibrate auto` (robot berputar sendiri) atau `calibrate manual` (robot diputar dengan tangan, timeout 15 detik).
- Tiap sensor dihitung ambangnya sendiri dari rata-rata max/min, **dijepit ke 3–18**, dan disimpan ke EEPROM.
- Tidak ada auto-kalibrasi saat boot secara default (`AUTO_CALIBRATE_ON_BOOT = false`).

### Tuning Tombol Onboard
- **Tombol 1 (pendek)**: `Kp ±1.0` atau `Kd ±0.5` tergantung `modeSetelKd`. **Tombol 1 (lama)**: saat berhenti toggle mode Kp/Kd; saat berjalan menghentikan robot.
- **Tombol 2 (pendek)**: `kecepatanDasarMaks ±10` (dijepit 60–150). **Tombol 2 (lama)**: simpan ke EEPROM dan mulai jalan.
- **LED RGB**: putih = persimpangan/start, magenta = garis hilang / mode setel Kd, kuning = kalibrasi, hijau berkedip 3× = kalibrasi selesai. Saat berhenti: magenta = mode Kd-tune, kuning = LEARNING, hijau = RACING, cyan = RC, mati = NORMAL.

### Keselamatan (Safety)
- **Baterai kritis**: berhenti semua fungsi jika tegangan ≤ ambang (diusulkan latch hingga power cycle).
- **Tilt (miring)**: matikan motor saat robot terangkat/terbalik (IMU roll/pitch > ambang).
- **Ultrasonik**: berhenti semua logika jalan jika objek depan ≤ 5 cm, prioritas tertinggi saat berjalan.
- **Deteksi macet**: robot dianggap macet jika IR beku + akselerasi stabil selama 3000 ms dan `|error| ≥ 2.0`; robot akan berhenti.
- **Lost-line timeout**: 2000 ms tanpa garis → berhenti + LED merah.
- Integral direset saat keluar dari lost-line recovery agar tidak ada "hutang" koreksi lama.

### Fitur Opsional (lewat `config.h`)
- `ENABLE_LAP_TIMER` — lap timer (start/finish line).
- `ENABLE_HEADING_CORRECTION` — koreksi heading berbasis gyro Z.
- `ENABLE_GAIN_SCHEDULING` — gain PID berbeda per kecepatan.
- `ENABLE_FUZZY_PID` — fuzzy adaptive PID.
- `ENABLE_ADAPTIVE_THRESHOLD` — ambang sensor dinamis menurut kecepatan.
- `ENABLE_PATH_PLANNER` — path planner A* (mode peta manual, bukan navigasi otonom; sangat RAM-intensive).
- `ENABLE_TELEMETRI` — dump CSV berkala via Serial Plotter.

## Persyaratan & Instalasi

### Hardware
- Board uKit Explore 2 (berbasis ATMega2560).
- Sensor IR kiri/kanan, sensor ultrasonik depan, sensor cahaya, sensor suara (tepuk tangan).
- Modul Bluetooth HM-10 atau built-in (jika `ENABLE_BLUETOOTH`).

### Software
- **Arduino IDE** dengan library `uKitExplore2` terinstal (tidak disertakan di repo).
- **Arduino CLI** (alternatif compile/upload tanpa GUI, lihat `BUILD.md`).
- Library bawaan `EEPROM.h`.
- CI GitHub Actions mengompilasi sketch utama + tiga test sketch (lihat `.github/workflows/compile.yml`).

## Konfigurasi

Semua nilai tunable ada di `LineFollowerFin/config.h`. Tidak ada `.env.example`; konfigurasi bersifat compile-time melalui konstanta C++.

| Kelompok | Contoh Parameter | Keterangan |
|----------|------------------|------------|
| PID | `Kp`, `Ki`, `Kd`, `BATAS_INTEGRAL` (50) | Gain PID dan anti windup |
| Kecepatan | `kecepatanDasarMaks`, `kecepatanDasarMin`, `SERVO_MAKS_DIPAKAI` (150), `SLEW_MAKS_PER_LOOP` (12) | Batas kecepatan servo dan slew limit |
| Turbo | `KECEPATAN_TURBO` (20), `LURUS_THRESHOLD` (2.0), `LURUS_TIME_REQUIRED` (500 ms) | Profil kecepatan lintasan lurus |
| Kalibrasi | `CALIB_USE_IMU`, `CALIB_TARGET_DERAJAT` (360), `CALIB_SPIN_TIMEOUT_MS` (8000) | Kalibrasi berbasis IMU |
| Safety | `TEGANGAN_MIN` (5.5), `TILT_MAKS_DERAJAT` (45), `JARAK_BERHENTI_CM` (5), `RECOVERY_TIMEOUT_MS` (2000) | Baterai, tilt, ultrasonik, lost-line |
| Sensor | `ALPHA_SENSOR` (0.65), `AMBANG_KIRI`, `AMBANG_KANAN`, `LIGHT_SENSOR_ID`, `ULTRASONIC_ID`, `SOUND_SENSOR_ID` | Filter dan ID sensor |
| Opsional | `ENABLE_LAP_TIMER`, `ENABLE_HEADING_CORRECTION`, `ENABLE_PATH_PLANNER`, dst. | Flag fitur opsional |
| EEPROM | `ADDR_EEPROM_CHECK` (0), `ADDR_KP` (4), `ADDR_MAP` (24), dst. | Alamat penyimpanan EEPROM |

Nilai `Kp`, `Ki`, `Kd`, `kecepatanDasarMaks`, `AMBANG_KIRI`, `AMBANG_KANAN` di-override oleh EEPROM saat valid. `kecepatanDasarMin` **tidak disimpan** ke EEPROM (selalu default dari `config.h`).

Penulisan EEPROM hanya dilakukan jika nilai berubah (`eepromWriteIfChanged` / template `eepromPutHemat`).

## Cara Menjalankan & Testing

### Compile & Upload (Arduino IDE)
1. Buka `LineFollowerFin/LineFollowerFin.ino`.
2. Pastikan library `uKitExplore2` terinstal.
3. Pilih board uKit Explore 2 / Arduino Mega2560.
4. Compile dan upload.

### Compile & Upload (Arduino CLI)
```bash
# Masuk ke folder sketch
cd LineFollowerFin

# Compile sketch utama (board ATMega2560)
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin.ino

# Upload ke port yang terdeteksi
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega LineFollowerFin.ino

# Serial monitor (baud 115200)
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

### Compile Test Sketch
Test sketch berada di `LineFollowerFin/tests/`; nama folder harus sama dengan nama `.ino`.

```bash
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin/tests/TestLapTimer
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin/tests/TestBluetooth
arduino-cli compile --fqbn arduino:avr:mega LineFollowerFin/tests/TestPathPlanner
```

Output compile sukses (semua fitur opsional OFF):
```
Sketch uses 94970 bytes (37%) of program storage space. Maximum is 253952 bytes.
Global variables use 5998 bytes (73%) of dynamic memory, leaving 2194 bytes for local variables. Maximum is 8192 bytes.
```

Lihat `BUILD.md` untuk panduan lengkap (install arduino-cli, board uKit, troubleshooting, CI/CD GitHub Actions).

## Struktur Direktori Utama

```
line-follower-ukit/
├── LineFollowerFin/              # Sketch Arduino (harus nama folder sama dengan .ino)
│   ├── LineFollowerFin.ino       # Objek, setup(), state machine loop()
│   ├── config.h                  # Semua nilai tunable (PID, speed, ambang, flag ENABLE_*)
│   ├── config.cpp
│   ├── status.h                  # Struct Status shared (jalan, modeSetelKd, recoveryMulai, dst.)
│   ├── fwd.h                     # Forward declarations (wajib untuk template eepromPutHemat)
│   ├── eeprom_layout.h           # Layout EEPROM versi struct + static_assert
│   ├── eeprom_layout (impl.)     # Helper eepromWriteIfChanged
│   ├── sensor_garis.h/.cpp       # Filter IR, ambang per-sensor, kalibrasi, baca EEPROM
│   ├── penggerak.h/.cpp          # Komputasi PID, deadband/slew, gerak motor servo, telemetry
│   ├── pengaman.h/.cpp           # Battery latch, tilt halt, ultrasonic stop, stuck detector, lampu otomatis
│   ├── antarmuka.h/.cpp          # Tombol onboard, tepuk tangan, parser perintah serial
│   ├── mapping.h/.cpp            # Mapper learning/racing untuk prediksi event persimpangan
│   ├── lap_timer.h/.cpp          # Lap timer opsional (start/finish line)
│   ├── bluetooth_ctrl.h/.cpp     # Kontrol/telemetri Bluetooth (opsional)
│   ├── rc_ctrl.h/.cpp            # Kontrol remote (opsional)
│   ├── heading_tracker.h/.cpp    # Koreksi heading IMU (opsional)
│   ├── path_planner.h/.cpp       # Path planner A* (opsional, root sketch folder)
│   ├── fuzzy_pid.h/.cpp          # Fuzzy adaptive PID (opsional)
│   ├── pid_controller.h/.cpp     # Kontroler PID dasar
│   ├── motor_driver.h/.cpp       # Driver motor servo
│   ├── speed_profile.h/.cpp      # Profil kecepatan
│   ├── ukit_wrapper.h            # Wrapper library uKit (digunakan file .cpp)
│   ├── ukit_extern.h             # Deklarasi eksternal uKit
│   ├── ukit_macros.h             # Macro uKit
│   ├── serial_silent.h           # Output serial conditional
│   ├── tests/
│   │   ├── TestLapTimer/TestLapTimer.ino
│   │   ├── TestBluetooth/TestBluetooth.ino
│   │   └── TestPathPlanner/TestPathPlanner.ino
│   └── TEST_GUIDE.md
├── docs/                         # Dokumentasi arsitektur, API, design decisions, ukit_explore
├── sim/                          # Simulator Python (robot, track, pid, sensors)
├── simulator/                    # Simulator alternatif Python
├── blockly/                      # Workspace Blockly (main.json)
├── .github/workflows/compile.yml # CI compile sketch utama + 3 test sketch
├── BUILD.md                      # Panduan compile/upload arduino-cli
├── AGENTS.md                     # Instruktur agen (struktur, konvensi, gotchas)
├── WIRING.md                     # Panduan rangkaian hardware
├── platformio.ini                # Konfigurasi PlatformIO (env mega)
└── README.md                     # File ini
```

### Catatan Penting
- `eepromPutHemat` adalah template di header; butuh **forward declaration eksplisit** di `fwd.h` sebelum `setup()`.
- Komentar dan serial print sebagian besar berbahasa Indonesia — pertahankan konsistensi.
- Sensor ID (cahaya/ultrasonik/suara) bisa berbeda tergantung wiring — sesuaikan di `config.h`.
- Fitur opsional default OFF untuk hemat RAM ATMega2560 (batas ~7 KB dynamic memory).
- Library uKit tidak di-vendor di repo; unduh dari sumber resmi (UBTEDU/uKitExplore-library).