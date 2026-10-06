# Line Follower UKIT

Robot line follower PID untuk board uKit Explore 2. Seluruh program ada dalam satu file: `LineFollowerFin.ino`.

## Kebutuhan

- Arduino IDE dengan library `uKitExplore2` terinstal (tidak disertakan di repo ini).
- Library bawaan `EEPROM.h`.
- Untuk verifikasi: compile `LineFollowerFin.ino` di Arduino IDE. Tidak ada automated test/lint/CI.

## Cara kerja singkat

1. Saat dinyalakan, robot membaca `Kp`, `Kd`, `kecepatanDasarMaks`, dan `AMBANG_BATAS` dari EEPROM (jika magic `123` ada di alamat 0), lalu kalibrasi sensor otomatis: berputar ±3 detik (LED kuning), ambang batas = rata-rata nilai maks/min sensor, dijepit ke 5–14, lalu disimpan ke EEPROM.
2. Saat berjalan (`loop()`), ada 3 kondisi:
   - **Persimpangan** (kedua sensor ≥ ambang): LED putih, maju aman lalu belok 90° kiri/kanan atau lurus mengikuti sisi yang nilainya lebih besar.
   - **Garis hilang** (kedua sensor < ambang): LED magenta, berputar di tempat ke arah sisi `lastError` terakhir.
   - **Tracer PID normal**: error = `(kiri − kanan) / 2`, dengan `I` dijepit ±50 dan kecepatan dasar adaptif (`maks − |PID| × 0,6`, dijepit ke `[min, maks]`).
3. Konvensi hardware: sensor kiri = `readInfraredDistance(2)`, kanan = `(1)`; servo 4+2 = sisi kiri (arah `1`), servo 1+3 = sisi kanan (arah `0`); rentang kecepatan 0–150.

## Tuning tanpa laptop (tombol onboard)

- **Tombol 1** tekan pendek: Kp ±1,0 (atau Kd ±0,5 bila `modeSetelKd` aktif). Tekan lama: saat berhenti beralih mode Kp/Kd, saat berjalan menghentikan robot.
- **Tombol 2** tekan pendek: kecepatan maks ±10 (dijepit 60–150). Tekan lama: simpan ke EEPROM dan mulai jalan.
- **LED RGB**: putih = persimpangan/start, magenta = garis hilang / mode setel Kd, kuning = kalibrasi, hijau berkedip 3× = kalibrasi selesai.

## Penyimpanan EEPROM

`Kp` (addr 4), `Kd` (addr 8), `kecepatanDasarMaks` (addr 12), `AMBANG_BATAS` (addr 16), dengan penanda `123` di addr 0. `Ki` dan `kecepatanDasarMin` tidak disimpan (selalu nilai default kode).

## Catatan untuk kontributor agen

Lihat `AGENTS.md` untuk detail polaritas motor, tata tombol/LED, dan batasan yang harus dijaga saat mengubah perilaku (mis. helper belok 90° menunggu garis dalam loop `while` yang blocking).
