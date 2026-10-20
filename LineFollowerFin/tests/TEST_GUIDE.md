# TEST SKETCHES - Panduan Validasi Hardware

## 📁 File Test yang Dibuat

| File | Fitur | Hardware Diperlukan |
|------|-------|---------------------|
| `TestLapTimer.ino` | Lap Timer | Hanya robot + trek dengan garis start/finish |
| `TestBluetooth.ino` | Bluetooth HM-10 | Robot + modul HM-10/HC-08 + HP |
| `TestPathPlanner.ino` | Path Planning A* | Robot + (opsional) ultrasonic |

---

## 🚀 Cara Test Masing-Masing

### 1. TestLapTimer.ino

**Persiapan:**
- Buat trek oval/lingkaran dengan **garis start/finish tebal** (2-3cm)
- Garis start/finish harus lebih tebal dari garis trek biasa
- Letakkan robot di garis start/finish

**Upload & Test:**
```
1. Upload TestLapTimer.ino ke uKit Explore 2
2. Buka Serial Monitor (115200 baud)
3. Tekan Tombol 2 LAMA (1+ detik) -> "TEST DIMULAI"
4. Robot jalan mengikuti garis (perlu PID manual atau push manual)
5. Setiap lewat start/finish -> "LAP X : Y.ZZs"
6. Perintah serial:
   - `lap`      -> summary
   - `lapr`     -> reset counter
   - `lapth 10` -> set threshold (butuh restart untuk apply)
```

**Tuning Threshold:**
- Jika false positive (deteksi lap di persimpangan biasa): **naikkan threshold** (12→14)
- Jika miss lap (lewat start/finish tapi tidak terdeteksi): **turunkan threshold** (12→8)
- Coba `LAP_MIN_LAP_MS` 2000-5000 tergantung ukuran trek

---

### 2. TestBluetooth.ino

**Wiring HM-10 ke uKit Explore 2:**
```
HM-10          uKit Explore 2 (Serial1)
VCC     ->     3.3V (JANGAN 5V!)
GND     ->     GND
TX      ->     RX1 (Pin 19)  -- butuh voltage divider 3.3V!
RX      ->     TX1 (Pin 18)  -- langsung OK (3.3V logic)
```

**Voltage Divider untuk TX HM-10 → RX1 uKit:**
```
HM-10 TX ----[1kΩ]----+----[2kΩ]---- GND
                      |
                      +---- uKit RX1 (Pin 19)
```

**Test Step-by-step:**
```
1. Upload TestBluetooth.ino
2. Buka Serial Monitor (115200)
3. Lihat: "HM-10 CONNECTED (OK)" atau "NO RESPONSE"
4. Pair HP ke HM-10 (Bluetooth settings -> "HMSoft" / "BT05", PIN: 0000/1234)
5. Buka app "Serial Bluetooth Terminal" (Android) / "BLE Terminal" (iOS)
6. Kirim "AT" -> harus balas "OK"
7. Test perintah:
   - `start`     -> "OK: Robot START"
   - `stop`      -> "OK: Robot STOP"
   - `status`    -> status koneksi
   - `telemetry on` -> streaming data tiap 500ms
   - `bat`       -> battery dummy
   - `help`      -> daftar command
```

**Troubleshooting:**
- **Tidak konek**: Cek wiring TX/RX terbalik? Voltage divider benar?
- **Garbled text**: Baud rate mismatch. HM-10 default 9600. Ubah di sketch: `BT_BAUD 9600`
- **AT tidak balas**: Coba `AT+RESET` atau `AT+BAUD8` (set 115200 permanen)

---

### 3. TestPathPlanner.ino

**Test Tanpa Hardware Ekstra:**
```
1. Upload TestPathPlanner.ino
2. Buka Serial Monitor (115200)
3. Lihat map 16x16 dengan robot di tengah (8,8)
4. Test command:
   - `goal 12 5`     -> set goal, auto find path
   - `map`           -> tampilkan map dengan path (+)
   - `obs 10 5`      -> tambah obstacle di (10,5)
   - `find`          -> cari path baru hindar obstacle
   - `auto`          -> simulasi robot bergerak ke goal
   - `sensor`        -> baca sensor IR real-time
```

**Integrasi Sensor (Optional):**
- Tambahkan ultrasonic di depan robot
- Saat `aman.halangan()` true -> `markObstacle(robotX+1, robotY)`
- Path planner akan menghindar obstacle otomatis

---

## 🔧 Debugging Tips

### Serial Monitor Output Terlalu Cepat?
- Tambah `delay(100)` di loop
- Atau kurangi `PRINT_INTERVAL` di TestLapTimer

### Memory Full (RAM)?
- TestPathPlanner pakai grid 16x16 = ~2KB RAM
- Jika error, kurangi `GRID_W`/`GRID_H` ke 12x12

### PID Belum Diintegrasikan?
- Test sketch **tidak** punya PID line following
- Robot harus didorong manual atau tambahkan PID sederhana
- Untuk test lengkap, gunakan `LineFollowerFin.ino` utama

---

## ✅ Checklist Validasi

### Lap Timer
- [ ] Deteksi start/finish akurat (tidak false positive di persimpangan)
- [ ] Lap time masuk akal (sesuai stopwatch manual)
- [ ] Counter lap increment benar
- [ ] `lap` / `lapr` / `lapth` command berfungsi

### Bluetooth
- [ ] HM-10 pair ke HP berhasil
- [ ] AT command balas "OK"
- [ ] Perintah `start`/`stop` diterima & dikonfirmasi
- [ ] Telemetry streaming stabil (tanah drop packet)
- [ ] Baud rate 115200 stabil (setelah `AT+BAUD8`)

### Path Planner
- [ ] Map render benar di Serial Monitor
- [ ] A* cari path ke goal tanpa obstacle
- [ ] Obstacle dihindari dengan benar
- [ ] Path length masuk akal (tidak zig-zag berlebihan)
- [ ] `auto` demo bergerak ke goal

---

## 📝 Catatan Penting

1. **Test sketches TIDAK punya safety features** (battery, tilt, ultrasonic stop) - hati-hati saat robot jalan
2. **Garis start/finish harus beda** dari garis trek biasa (lebih tebal/panjang)
3. **HM-10 logic 3.3V** - wajib voltage divider untuk RX uKit
4. **Path planner butuh odometry/IMU** untuk posisi real-time - saat ini pakai simulasi
5. **Integrasi ke main code** butuh tuning parameter & testing bersamaan

---

## 📞 Butuh Bantuan?

Jika ada error compile/upload atau behavior aneh:
1. Copy paste error message
2. Foto wiring (khusos Bluetooth)
3. Video behavior robot
4. Serial monitor log

Saya bantu debug! 🛠