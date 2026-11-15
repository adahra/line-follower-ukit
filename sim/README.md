# Sim — simulator line follower (Python)

Simulator berbasis Python untuk sketsa `LineFollowerFin`: logika kontrolnya
adalah **port langsung** dari kode C++ (PID adaptif, turbo, deadband servo,
slew limiter, recovery lost-line, manuver persimpangan 90°), diberi model
fisika differential-drive dan model sensor IR reflektif.

## Pakai

```bash
python3 simulate.py                          # headless + ringkasan lap
python3 simulate.py --durasi 90              # 90 detik simulasi
python3 simulate.py --kp 3 --kd 1.0          # eksperimen gain
python3 simulate.py --render out.mp4         # run + rekam video 2x speed
SIM_CSV=out.csv python3 simulate.py          # dump telemetri CSV
# format CSV sama dengan telemetri robot: t,err,pid,dasar,kiri,kanan,senK,senN
```

Butuh: `numpy`, `matplotlib` (untuk render video; ffmpeg tersedia via imageio-ffmpeg writer bawaan matplotlib).

## Isi

- `track.py` — geometri lintasan: loop tertutup oval bergelombang
  (r = 1.4 + 0.42·sin 3θ, ~10.4 m) + model sensor: bacaan 0–20, penuh saat
  sensor di atas pita 1.9 cm, turun ke lantai 1 di luar transisi 1.5 cm.
- `robot.py` — port `Penggerak::hitung()` (PID adaptif + turbo + deadband +
  slew) dan `SensorGaris` (filter eksponensial). Konstanta disalin dari
  `config.h`/`config.cpp` (Kp=5, Ki=0.1, Kd=1.5, speed 110/55, ambang 8).
- `simulate.py` — loop utama setia pada state machine `.ino`:
  - KONDISI A: kedua sensor ≥ ambang → maju 150 ms → keputusan L/R/lurus →
    belok 90° blocking (timeout 1.5 s).
  - KONDISI B: kedua sensor < ambang → pivot non-blocking ke arah `lastError`,
    berhenti total setelah 2 s.
  - KONDISI C: PID tracer dengan interval 10 ms, lap timer min 3 s/lap.

## Hasil contoh (lintasan demo)

| Kp | Kd | best lap |
|----|----|----------|
| 3  | 1.0 | 24.3 s |
| 5 (default) | 1.5 | 25.1 s |
| 7  | 1.5 | 26.1 s |
| 9  | 1.5 | tidak stabil (lap 76 s / garis hilang) |

Kp terlalu tinggi = overcorrect → lambat, lalu tidak stabil. Cocok dipakai
untuk menyaring kandidat gain sebelum diuji di robot.

## Perbedaan vs robot asli (penting)

- Sensor: model pita + ramp adalah **perkiraan**; respons & posisi sensor asli
  uKit tidak diketahui persis. Gain optimal di sim belum tentu optimal di robot.
- Fisika: tanpa slip, inersia, atau deadband mekanis; kecepatan servo dipetakan
  linear 0.004 m/s per unit (150 = 0.6 m/s), wheelbase 16 cm — semua perkiraan.
- Safety (ultrasonik, tilt, baterai, macet) dan Bluetooth tidak disimulasikan.
- Belokan 90° dimodelkan pivot in-place dengan laju putar aproksimasi; robot
  asli memakai pola servo 4 roda berbeda.
- Manuver persimpangan (A) tidak pernah terpicu di lintasan demo (tanpa garis
  melintang); logikanya tetap di-port. Catatan: keputusan belok di persimpangan
  memakai perbandingan bk vs bn yang rawan simetris — sejalan dengan temuan
  review #5 bahwa arah belok perlu diuji di trek.
