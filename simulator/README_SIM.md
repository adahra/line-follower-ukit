# Simulator LineFollowerFin (Python + Tkinter)

Simulator berbasis **logika** (bukan emulasi hex) untuk LineFollowerFin. Meniru
perilaku `LineFollowerFin.ino`: state machine, PID, deadband/slew servo, dan
konvensi sensor/motor asli.

## Menjalankan

Dari root repo:

```bash
python3 simulator/main.py
```

Perlu: Python 3 + Tkinter + Pillow + NumPy (sudah tersedia di lingkungan ini).
Tidak perlu library Arduino/uKitExplore2.

## Struktur file

- `config_sim.py` — nilai default meniru `config.h`/`config.cpp` (Kp, Ki, Kd,
  kecepatanDasarMaks/Min, FAKTOR_REM_ADAPTIF 0.35, BATAS_INTEGRAL 50, deadband
  25, slew ±14, ambang 8, recovery timeout 2000 ms, dll).
- `pid.py` — implementasi PID persis: `error=(kiri-kanan)/2`, `I` clamp ±50,
  `kecepatanDasar = maks - abs(PID)*0.35` dipenjit [min, maks], deadband 25,
  slew ±`SLEW_MAKS_PER_LOOP`.
- `sensors.py` — sensor IR simulasi: baca piksel hitam/putah di bawah posisi
  sensor kiri (ID 2) dan kanan (ID 1) dengan filter eksponensial.
- `track.py` — lintasan: load gambar (PNG/JPG), gambar manual klik-drag di
  canvas, atau garis default; `nilai_piksel()` mengembalikan kekelamatan.
- `robot.py` — model robot + state machine (persimpangan / lost-line / PID
  normal), kinematika diferensial, kalibrasi simulasi.
- `main.py` — UI Tkinter: canvas, panel debug, slider tuning live,
  tombol Start/Stop/Reset/Kalibrasi.
- `README_SIM.md` — berkas ini.

## Kontrol UI

- **Mode operasi** (dropdown): NORMAL, LEARNING, RACING (disimpan di
  `robot.mode`; perilaku inti sama untuk ke-3 mode, seperti asli tanpa waypoint).
- **Start / Stop**: jalankan atau hentikan robot.
- **Reset**: reset posisi, PID, filter sensor, dan state.
- **Kalibrasi**: putar 3 detik lalu hitung `AMBANG_KIRI`/`AMBANG_KANAN`
  dari min/max bacaan, dikunci ke 3–18.
- **Track**: muat gambar PNG/JPG, bersihkan, simpan, atau klik-drag di canvas
  untuk menggambar garis hitam manual. Garis default tersedia.
- **Tuning live**: slider Kp, Ki, Kd, Speed Maks, Speed Min, Ambang Kiri/Kanan
  memperbarui `config_sim` seketika saat simulasi berjalan.

## Panel debug (label, bahasa Indonesia)

State, mode, nilai sensor kiri(ID2)/kanan(ID1), error, P/I/D, PID, kecepatan
dasar, speed motor kiri/kanan, dan timer recovery.

## Konvensi yang dipertahankan dari robot asli

- Sensor kiri = ID 2, sensor kanan = ID 1.
- Motor kiri = servo 4 + 2 (dir 1), motor kanan = servo 1 + 3 (dir 0).
- PID interval konstan 10 ms; `I` dijepit ±50; kecepatan dasar adaptif
  `maks - abs(PID)*0.35`; output dibatasi 0–150 (derate servo); deadband 25;
  slew ±14 per loop.
- Persimpangan: kedua sensor >= ambang -> maju 150 ms, keputusan belok ke arah
  sensor yang lebih gelap, else lurus.
- Lost line: kedua sensor < ambang -> putar ke arah `lastError`; timeout
  2000 ms -> berhenti.

Catatan: ini simulasi logika di PC, bukan emulasi instruksi AVR/hex.
