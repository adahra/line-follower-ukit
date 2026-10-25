# WIRING.md — Pinout uKit Explore 2 untuk LineFollowerFin

> **Catatan**: ID di bawah mengacu pada `config.h`. Sesuaikan jika wiring beda.

## Servo Motor 360° (4 buah)
| Servo ID | Posisi | Sisi | Arah `setServoTurn(id, dir, speed)` |
|----------|--------|------|-------------------------------------|
| 1        | Kanan depan  | Kanan | `dir = 0` (maju) |
| 2        | Kiri belakang | Kiri  | `dir = 1` (maju) |
| 3        | Kanan belakang | Kanan | `dir = 0` (maju) |
| 4        | Kiri depan   | Kiri  | `dir = 1` (maju) |

**Mapping di kode** (`penggerak.cpp:79-83`):
```cpp
setServoTurn(4, 1, speedKiri);  // Kiri depan
setServoTurn(2, 1, speedKiri);  // Kiri belakang
setServoTurn(1, 0, speedKanan); // Kanan depan
setServoTurn(3, 0, speedKanan); // Kanan belakang
```
- **Speed range**: 0–150 (derate dari 0–255 docs, agar stabil)
- **Deadband**: 1–24 → dipaksa ke 25 (`SERVO_MIN_EFEKTIF`)

## Sensor Infrared Garis (2 buah)
| Sensor ID | Posisi | Variabel | Fungsi baca |
|-----------|--------|----------|-------------|
| 1         | Kanan  | `bacaKanan` | `readInfraredDistance(1)` |
| 2         | Kiri   | `bacaKiri`  | `readInfraredDistance(2)` |

**Konvensi error**: `error = (kiri - kanan) / 2.0` → positif = belok kanan, negatif = belok kiri

**Ambang per sensor** (EEPROM addr 20/22, default 8):
- `AMBANG_KIRI`  — sensor ID 2
- `AMBANG_KANAN` — sensor ID 1

## Sensor Ultrasonic (Halangan Depan)
| Config | Nilai | Keterangan |
|--------|-------|------------|
| `ULTRASONIC_ID` | 1 | ID terverifikasi; sensor ultrasonic menghadap depan |
| `JARAK_BERHENTI_CM` | 5 | Stop bila ≤ 5 cm |
| `CEK_HALANGAN_MS` | 100 | Throttle ping |

## Sensor Cahaya (Auto Headlight)
| Config | Nilai | Keterangan |
|--------|-------|------------|
| `LIGHT_SENSOR_ID` | 1 | ID terverifikasi; port sensor cahaya |
| `EYELIGHT_KIRI` | 1 | Eye lamp kiri |
| `EYELIGHT_KANAN` | 2 | Eye lamp kanan |
| `AMBANG_GELAP_NYALA` | 100 lux | Nyala putih di bawah ini |
| `AMBANG_GELAP_MATI` | 150 lux | Histeresis mati di atas ini |

## Sensor Suara (Clap Toggle)
| Config | Nilai |
|--------|-------|
| `SOUND_SENSOR_ID` | 1 |
| `TEPUK_AMBANG` | 500 (intensitas 0–1023) |
| `TEPUK_LEPAS` | 350 (histeresis) |

## Tombol Onboard (2 buah)
| Tombol | Fungsi |
|--------|--------|
| 1 (pendek) | Kp ±1.0 / Kd ±0.5 (mode `modeSetelKd`) |
| 1 (lama) | Toggle mode Kp/Kd (berhenti), Stop (jalan) |
| 2 (pendek) | Speed maks ±10 (clamp 60–150) |
| 2 (lama) | Save EEPROM + Start (kalau berhenti) |
| 1+2 bersamaan (berhenti) | Kalibrasi manual sensor (putar 3 detik) |

## RGB LED (Status Indikator)
| Warna | Kondisi |
|-------|---------|
| Putih (255,255,255) | Persimpangan / Start |
| Magenta (255,0,255) | Lost-line recovery / Mode setel Kd |
| Kuning (255,255,0) | Kalibrasi berlangsung |
| Hijau berkedip 3× | Kalibrasi selesai |
| Merah (255,0,0) | Error: baterai lemah / miring / halangan / macet |

## EEPROM Layout
| Addr | Tipe | Variabel | Catatan |
|------|------|----------|---------|
| 0    | byte | Magic `123` | Valid marker |
| 4    | float | `Kp` | |
| 8    | float | reserved / `Ki` lama | Tidak dipakai untuk persistence aktif |
| 12   | float | `Kd` | |
| 16   | int | `kecepatanDasarMaks` | |
| 20   | int | `AMBANG_KIRI` | |
| 22   | int | `AMBANG_KANAN` | |

> `Ki` dan `kecepatanDasarMin` **tidak** disimpan — pakai default kode. EEPROM hanya ditulis bila nilai berubah.

## IMU Onboard
- `IMU::init()` di `setup()`
- `IMU::read()` tiap loop (throttle 50ms)
- `getRoll()`, `getPitch()` — tilt deteksi (>45° = stop)
- `getRawAccelX/Y/Z()` — stuck detector (magnitude stabil + IR beku)

## Baterai
- `readBatteryVoltage()` — 0–8.4V
- `TEGANGAN_MIN` = 5.5V → latch stop, butuh power cycle

## Serial Commands (baru)
Buka Serial Monitor (115200 baud), ketik:
```
kp 22.5       # set Kp
ki 0.02       # set Ki
kd 1.8        # set Kd
speed 120     # set speed maks (60-150)
speedmin 50   # set speed min
ambang 10     # set ambang kiri & kanan
ambangk 9     # set ambang kiri saja
ambangn 11    # set ambang kanan saja
calibrate     # kalibrasi sensor (harus berhenti)
save          # simpan ke EEPROM
load          # muat dari EEPROM & cetak
start         # mulai jalan
stop          # berhenti
help          # tampilkan bantuan
```

## Verifikasi Cepat
1. Colok USB → buka Serial Monitor 115200
2. Lihat output boot: `EEPROM valid: YA/TIDAK`, nilai Kp/Ki/Kd/Speed/Ambang
3. Ketik `help` → cek command parser jalan
4. Tekan tombol 1+2 bersamaan (robot berhenti) → kalibrasi → lihat LED kuning → hijau 3×
5. Tekan tombol 2 lama → start → robot jalan PID