# Line Follower UKIT

Robot line follower PID untuk board uKit Explore 2. Seluruh program ada dalam satu file: `LineFollowerFin/LineFollowerFin.ino` (nama file harus sama dengan folder, syarat Arduino IDE).

## Kebutuhan

- **Arduino IDE** dengan library `uKitExplore2` terinstal (tidak disertakan di repo ini).
- **Atau arduino-cli** (lihat [BUILD.md](docs/BUILD.md) untuk compile/upload via CLI).
- Library bawaan `EEPROM.h`.
- Untuk verifikasi: compile `LineFollowerFin/LineFollowerFin.ino` di Arduino IDE. Tidak ada automated test/lint/CI.

## Cara kerja singkat

1. Saat dinyalakan, robot membaca `Kp`, `Kd`, `kecepatanDasarMaks`, dan ambang kiri/kanan dari EEPROM (jika magic `123` ada di alamat 0). Tidak ada kalibrasi otomatis — bila perlu, tekan **kedua tombol bersamaan** saat berhenti: robot berputar ±3 detik (LED kuning), tiap sensor dihitung ambangnya sendiri dari rata-rata maks/min-nya, dijepit ke 5–14, lalu disimpan ke EEPROM.
2. Saat berjalan (`loop()`), ada 3 kondisi:
   - **Persimpangan** (kedua sensor ≥ ambang): LED putih, maju aman lalu belok 90° kiri/kanan atau lurus mengikuti sisi yang nilainya lebih besar.
   - **Garis hilang** (kedua sensor < ambang): LED magenta, berputar di tempat ke arah sisi `lastError` terakhir.
   - **Tracer PID normal**: error = `(kiri − kanan) / 2`, dengan `I` dijepit ±50 dan kecepatan dasar adaptif (`maks − |PID| × 0,6`, dijepit ke `[min, maks]`).
3. Konvensi hardware: sensor kiri = `readInfraredDistance(2)`, kanan = `(1)`; servo 4+2 = sisi kiri (arah `1`), servo 1+3 = sisi kanan (arah `0`); kecepatan di-derate 0–150 (docs membolehkan 0–255).
4. Fitur keselamatan: berhenti otomatis bila halangan depan ≤ 5 cm (sensor ultrasonic ID 1, prioritas tertinggi); eye lamp menyala putih otomatis saat gelap (< 100 lux, mati > 150 lux) memakai sensor cahaya ID 1.

## Tuning tanpa laptop (tombol onboard)

- **Tombol 1** tekan pendek: Kp ±1,0 (atau Kd ±0,5 bila `modeSetelKd` aktif). Tekan lama: saat berhenti beralih mode Kp/Kd, saat berjalan menghentikan robot.
- **Tombol 2** tekan pendek: kecepatan maks ±10 (dijepit 60–150). Tekan lama: simpan ke EEPROM dan mulai jalan.
- **LED RGB**: putih = persimpangan/start, magenta = garis hilang / mode setel Kd, kuning = kalibrasi, hijau berkedip 3× = kalibrasi selesai.

## Penyimpanan EEPROM

Magic `123` disimpan di addr 0. Data konfigurasi aktif:

- `Kp`: addr 4
- `Kd`: addr 12
- `kecepatanDasarMaks`: addr 16
- `AMBANG_KIRI`: addr 20
- `AMBANG_KANAN`: addr 22

`Ki` dan `kecepatanDasarMin` tidak disimpan; keduanya kembali ke nilai default kode setelah restart. Penulisan memakai `eepromPutHemat`, jadi EEPROM hanya ditulis bila nilai berubah.

## Fitur tambahan

- Bluetooth aktif melalui `BluetoothCtrl` bila `ENABLE_BLUETOOTH = true`.
- Lap timer aktif bila `ENABLE_LAP_TIMER = true`.
- Mode `MODE_LEARNING` merekam event persimpangan melalui `Mapper`.
- Mode `MODE_RACING` memberi prediction berbasis waktu dan feedback LED; prediction belum mengubah keputusan belok utama.
- Path planner tersedia sebagai kode opsional, tetapi default nonaktif melalui `ENABLE_PATH_PLANNER = false`.

## Catatan untuk kontributor agen

Lihat `AGENTS.md` untuk detail polaritas motor, tata tombol/LED, dan batasan yang harus dijaga saat mengubah perilaku (mis. helper belok 90° menunggu garis dalam loop `while` yang blocking).
