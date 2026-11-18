# Referensi Cepat - LineFollowerFin

### Referensi perintah yang sering digunakan

## Command Line

**Compile & Upload:**
```bash
# Arduino IDE:
File -> Upload to Board -> LineFollowerFin

# Arduino CLI:
arduino-cli compile --fqtn LineFollowerFin --upload-port /dev/ttyUSB0
arduino-cli upload --fqtn LineFollowerFin --port /dev/ttyUSB0
```

**Debugging & Serial:**
```bash
# Monitor serial (115200 baud):
picocom -b /dev/ttyUSB0 -r 115200

# Jalankan dengan debugging:
arduino-cli compile --fqtn LineFollowerFin --debug-port /dev/ttyUSB0
```

**Kalibrasi & Tuning:**
```bash
# Kalibrasi otomatis (mode manual):
arduino-cli upload --fqtn LineFollowerFin --port /dev/ttyUSB0 --program
# Tekan 2 tombol bersamaan saat robot berhenti

# Kirim command serial untuk tunings (di terminal serial):
kp 1.2
ki 1.0
kd 0.8
speed 100
ambang 15
calibrate
save
help
```

**Mode Operasi:**
```bash
# Jalankan:
arduino-cli upload --fqtn LineFollowerFin --port /dev/ttyUSB0 --program
# Di terminal serial:
start

# Berhenti:
stop

# Mode pembelajaran (waypoint recording):
start
# Di terminal serial:
mode learning
# Lakukan gerakan robot (belok kiri/kanan/straight)
# Ulangi untuk waypoint

# Simpan peta waypoint:
save

# Mulai balapan (racing):
start
# Di terminal serial:
mode racing
# Prediksi racing aktif
```

**Fitur Opsional:**
```bash
# Aktifkan Bluetooth:
# Di config.h:
#define ENABLE_BLUETOOTH true

# Aktifkan RC:
# Di config.h:
#define ENABLE_RC true

# Aktifkan Fuzzy PID:
# Di config.h:
#define ENABLE_FUZZY_PID true

# Aktifkan Heading Correction:
# Di config.h:
#define ENABLE_HEADING_CORRECTION true

# Aktifkan Path Planner:
# Di config.h:
#define ENABLE_PATH_PLANNER true
```

## Daftar Perintah

| Perintah | Deskripsi |
|---------|-------------| 
| `start` | Mulai robot, reset semua state |
| `stop` | Hentikan robot, reset state |
| `tune` | Aktifkan mode tuning Kp/Kd |
| `help` | Tampilkan daftar command yang tersedia |
| `calibrate` | Jalankan kalibrasi otomatis sensor |
| `save` | Simpan semua konfigurasi ke EEPROM |
| `mode learning` | Set mode pembelajaran (waypoint recording) |
| `mode racing` | Set mode racing (prediction based) |
| `mode rc` | Set mode RC control |
| `reset` | Reset semua modul (melalui Bluetooth) |
| `kp X` | Set Kp = X |
| `ki X` | Set Ki = X |
| `kd X` | Set Kd = X |
| `speed X` | Set speed max = X |
| `ambang X` | Set ambang sensor = X |

## Timer Interval

| Timer | Nilai | Pengaruh |
|------|--------|------------|
| `PID_INTERVAL_MS` | 10ms | Interval PID loop |
| `CEK_CAHAYA_MS` | 200ms | Throttle lampu gelap |
| `CEK_HALANGAN_MS` | 100ms | Throttle ultrasonic |
| `CEK_BATERAI_MS` | 500ms | Throttle pembacaan baterai |
| `CEK_IMU_MS` | 50ms | Throttle pembacaan IMU |
| `TEPUK_COOLDOWN_MS` | 800ms | Cooldown tepuk tangan |
| `COOLDOWN_TOMBOL_MS` | 300ms | Cooldown tombol |

## GPIO & Port Mapping

| Komponen | Pin | Keterangan |
|----------|-----|-------------|
| Sensor IR Kiri | ID 2 | Port uKit Explore 2 |
| Sensor IR Kanan | ID 1 | Port uKit Explore 2 |
| Sensor Cahaya | ID 1 | Port opsional |
| Ultrasonic | ID 1 | Port depan |
| Sound | ID 1 | Port opsional |
| Bluetooth | Serial1 | uKit built-in BT |
| RC Throttle | Pin 2 | PWM/PPM input |
| RC Steering | Pin 3 | PWM/PPM input |

## EEPROM Layout

| Alamat | Ukuran | Isi | Keterangan |
|---------|--------|-----|-------------|
| 3 | 1 byte | `77` | Guard uKit |
| 0 | 1 byte | `123` | Magic validitas |
| 4-15 | 12 bytes | Kp, Ki, Kd | PID gains |
| 16-17 | 2 bytes | kecepatanDasarMaks | Speed max |
| 18-19 | 2 bytes | AMBANG_KIRI | Ambang kiri |
| 20-21 | 2 bytes | AMBANG_KANAN | Ambang kanan |
| 24-266 | 242 bytes | Waypoint | Peta waypoint (30 x 8) |

## Status Mode

| Mode | Value | Keterangan |
|------|--------|-------------|
| `MODE_NORMAL` | 0 | Tracer garis normal |
| `MODE_LEARNING` | 1 | Rekam waypoint |
| `MODE_RACING` | 2 | Prediksi berdasarkan waypoint |
| `MODE_RC` | 3 | Remote control |

## SubState

| SubState | Value | Keterangan |
|----------|--------|-------------|
| `SUBSTATE_TRACKING` | 0 | Normal PID tracer |
| `SUBSTATE_INTER_CREEP` | 1 | Creep melewati persimpangan |
| `SUBSTATE_TURNING` | 2 | Belok 90° non-blocking |

## Komentar Penting

- **Non-blocking:** Semua loop punya throttle → tetap bisa melacak garis
- **Latching:** Kondisi darurat (baterai lemah) punya latch → hanya reset saat restart power
- **Priority:** Ultrasonic > kemiringan > baterai lemah > macet
- **Kalibrasi:** Mode manual butuh rotasi 360° berbasis gyro
- **Adaptive threshold:** Ambang dinamis berdasarkan kecepatan
- **Heading correction:** Koreksi berbasis gyro Z di atas PID
- **Fuzzy PID:** Adaptive PID gains berdasarkan error
- **RC:** Pengambil alih penuh kontrol saat diaktifkan
- **Bluetooth:** Telemetry & serial command via Bluetooth

## Troubleshooting

### Robot tidak melacak garis
1. Periksa kalibrasi sensor (`calibrate`)
2. Periksa ambang (`ambang 15`)
3. Periksa PID gains (`kp 1.2 ki 1.0 kd 0.8`)
4. Periksa sensor kotor/cahaya
5. Reset EEPROM (`save` setelah kalibrasi)

### Robot berhenti secara tak terduga
1. Periksa baterai (`bateraiKritis`)
2. Periksa kemiringan (`miring`)
3. Periksa halangan ultrasonic
4. Periksa macet (`macet`)
5. Reset semua (`reset`)

### Mode pembelajaran tidak merekam
1. Pastikan `status.jalan = true`
2. Gunakan command `mode learning`
3. Lakukan gerakan belok kiri/kanan/straight
4. Gunakan `save` untuk simpan

### Bluetooth tidak terhubung
1. Periksa wiring (Serial1)
2. Gunakan `begin()` dengan tipe BT_HM10
3. Periksa baud rate (115200)
4. Reset robot

### Kalibrasi macet
1. Gunakan mode manual (putar robot dengan tangan)
2. Gunakan gyro (CALIB_USE_IMU = true)
3. Periksa timeout (CALIB_FALLBACK_MS)
4. Pastikan tidak ada halangan

