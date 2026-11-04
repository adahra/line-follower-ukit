# Serial Command — Fuzzy PID

Dokumen ini adalah referensi praktis untuk mengaktifkan / menonaktifkan **Fuzzy Adaptive PID** lewat perintah serial (USB Serial Monitor). Untuk penjelasan arsitektur fuzzy lengkap (membership function, tabel aturan, inferensi), lihat [`FUZZY_PID.md`](./FUZZY_PID.md).

---

## 1. Prasyarat

Fuzzy PID bersifat opsional. Sebelum bisa dipakai lewat serial, dua syarat compile/runtime harus terpenuhi:

| Syarat | Lokasi | Default | Catatan |
|---|---|---|---|
| `ENABLE_FUZZY_PID = true` | `config.h` | `false` | Master switch compile-time. Jika `false`, class `FuzzyPID` tidak dikompilasi dan perintah `fuzzy` hanya mencetak pesan nonaktif. |
| `ENABLE_SERIAL_MODE = true` | `config.h` | `true` | Mengaktifkan `ui.cekSerial()` di `loop()`. Jika `false`, parser serial tidak jalan sama sekali. |
| Serial Monitor 115200 baud | — | — | Baud rate default uKit Explore 2 (lihat `BUILD.md`). |

Aktifkan fuzzy di `config.h`:

```cpp
#define ENABLE_FUZZY_PID true
```

Lalu compile & upload ulang. Tanpa langkah ini, perintah `fuzzy on` tidak akan berpengaruh — robot hanya menjawab:

```
Fuzzy PID tidak aktif (ENABLE_FUZZY_PID=false).
```

---

## 2. Perintah

Robot membaca serial di `loop()` lewat `Antarmuka::cekSerial()` → `parseCommand()`. Input otomatis di-lowercase dan trailing `\r`/`\n`/spasi dipangkas, jadi `FUZZY ON`, `Fuzzy On`, dan `fuzzy on` semua diterima.

| Perintah | Aksi | Output Serial |
|---|---|---|
| `fuzzy on` | Aktifkan fuzzy adaptive gains | `Fuzzy PID: ON (adaptive gains aktif)` |
| `fuzzy off` | Nonaktifkan, kembali ke gain statis | `Fuzzy PID: OFF (static gains dari config/EEPROM)` |
| `fuzzy` | Cek status (tanpa argumen) | `Fuzzy PID: ON`/`OFF` + `Penggunaan: fuzzy on \| fuzzy off \| fuzzy` |

Sumber kode (`antarmuka.cpp:375-391`):

```cpp
} else if (strcmp(key, "fuzzy") == 0) {
#if ENABLE_FUZZY_PID
  if (strcmp(valStr, "on") == 0) {
    gerak.setFuzzyEnabled(true);
    Serial.println(F("Fuzzy PID: ON (adaptive gains aktif)"));
  } else if (strcmp(valStr, "off") == 0) {
    gerak.setFuzzyEnabled(false);
    Serial.println(F("Fuzzy PID: OFF (static gains dari config/EEPROM)"));
  } else {
    Serial.print(F("Fuzzy PID: "));
    Serial.println(gerak.isFuzzyEnabled() ? F("ON") : F("OFF"));
    Serial.println(F("Penggunaan: fuzzy on | fuzzy off | fuzzy"));
  }
#else
  Serial.println(F("Fuzzy PID tidak aktif (ENABLE_FUZZY_PID=false)."));
#endif
}
```

---

## 3. Yang Terjadi di Balik Layar

`fuzzy on` memanggil `Penggerak::setFuzzyEnabled(true)` yang mengeset flag `FuzzyPID::enabled = true`. Setelah itu, setiap siklus PID di `Penggerak::hitung()` (interval `PID_INTERVAL_MS` = 10 ms) mengeksekusi:

1. Hitung `error` dan `delta_error` (`D = error - lastError`).
2. Panggil `fuzzyPid.compute(error, D, dKp, dKd, dKi)`.
3. Hitung gain efektif: `KpEff = Kp + dKp`, `KdEff = Kd + dKd`, `KiEff = Ki + dKi`.
4. Clamp gain efektif ≥ 0 (tidak boleh negatif).
5. `PID_value = (KpEff·P) + (KiEff·I) + (KdEff·D)` diterapkan ke motor.

Saat `fuzzy off`, `compute()` langsung return 0 — gain efektif sama dengan base gain (`Kp`, `Kd`, `Ki`) tanpa overhead inferensi.

---

## 4. Status via Perintah Lain

- **`status`** — menampilkan status lengkap. Baris PID mencakup flag fuzzy bila `ENABLE_FUZZY_PID` aktif:
  ```
  Kp: 22.0 | Kd: 1.8 | Ki: 0.1 | SpeedMaks: 120 | Dasar: 100 | Fuzzy: ON
  ```
- **`help`** — daftar perintah mencantumkan:
  ```
  fuzzy on|off   : aktifkan/nonaktifkan/cek Fuzzy adaptive PID
  ```

---

## 5. Penting: Tidak Persisten & Tidak Ada Tombol Fisik

- **Status tidak disimpan ke EEPROM.** Flag `enabled` hanya ada di RAM. Setiap power cycle / reboot, fuzzy kembali OFF. Jalankan `fuzzy on` lagi setiap boot.
- **Tidak ada tombol fisik untuk fuzzy.** Kode `cekTombol()` tidak memiliki handler fuzzy — sengaja, karena toggle tombol untuk fuzzy dianggap kurang andal (lihat komentar `antarmuka.cpp:70`: *"Fuzzy toggle lewat serial command 'fuzzy on/off' (lebih andal)."*). Serial adalah satu-satunya cara runtime.
- Yang persisten ke EEPROM hanya **base gain** (`Kp`, `Ki`, `Kd`, `kecepatanDasarMaks`, ambang, parameter heading/adaptive) lewat perintah `save`. Delta fuzzy (`dKp`/`dKd`/`dKi`) tidak pernah disimpan.

---

## 6. Bluetooth

Perintah `fuzzy` **tidak** tersedia via Bluetooth saat ini. `BluetoothCtrl::handleBTCommand()` (lihat `bluetooth_ctrl.cpp`) tidak memiliki case `fuzzy`, dan `commandCallback` yang meneruskan perintah tak dikenal ke parser serial standar **tidak di-wire** di `setup()`. Akibatnya, `fuzzy on` lewat Bluetooth akan menjawab `ERR: Unknown command`.

Bluetooth hanya menangani subset: `start`/`stop`/`mode`/`kp`/`kd`/`ki`/`speed`/`save`/`load`/`pid`/`bt ...`. Untuk fuzzy, gunakan USB Serial Monitor.

---

## 7. Contoh Sesi

Hubungkan USB, buka Serial Monitor 115200, newline/CR+LF boleh:

```
> help
(help command parser fuzzy on|off tercantum)

> fuzzy
Fuzzy PID: OFF
Penggunaan: fuzzy on | fuzzy off | fuzzy

> fuzzy on
Fuzzy PID: ON (adaptive gains aktif)

> status
...
Kp: 22.0 | Kd: 1.8 | Ki: 0.1 | SpeedMaks: 120 | Dasar: 100 | Fuzzy: ON
 --- AKHIR STATUS ---

> start
Serial: JALAN
(berjalan; gain efektif kini adaptif terhadap error & delta error)

> fuzzy off
Fuzzy PID: OFF (static gains dari config/EEPROM)

> stop
Serial: BERHENTI
```

---

## 8. Rekomendasi Penggunaan

1. **Tuning base gain dulu.** Fuzzy bekerja sebagai *delta* di atas base. Sebelum `fuzzy on`, pastikan `Kp`/`Kd`/`Ki` sudah wajar (via tombol, serial `kp`/`kd`/`ki`, atau `tune`). Base yang buruk + fuzzy tetap buruk.
2. **Bandingkan A/B.** Jalankan lintasan yang sama dengan `fuzzy off` lalu `fuzzy on`, amati respons di tikungan tajam (|error| besar) dan di lurus (osilasi). Fuzzy menambah Kp saat |error| besar dan menambah Kd saat osilasi.
3. **Pakai saat lomba.** Karena tidak persisten, jangan lupa kirim `fuzzy on` setelah robot nyala sebelum start.
4. **Jangan ganti base gain tanpa `save`.** Base `Kp`/`Kd` dari `kp`/`kd` serial hanya di RAM sampai `save`. Delta fuzzy tidak butuh simpan (selalu dihitung ulang).

---

## 9. Troubleshooting

| Gejala | Penyebab | Solusi |
|---|---|---|
| `Fuzzy PID tidak aktif (ENABLE_FUZZY_PID=false).` | Fuzzy tidak di-enable saat compile | Set `ENABLE_FUZZY_PID true` di `config.h`, recompile & upload |
| Perintah `fuzzy` tidak merespon sama sekali | `ENABLE_SERIAL_MODE = false` atau serial tidak terhubung | Pastikan `ENABLE_SERIAL_MODE true`; cek koneksi & baud 115200 |
| Fuzzy ON tapi respons sama dengan OFF | Base gain nol / tidak ada error signifikan | Tuning base gain dulu; fuzzy hanya delta kecil (±5/±2/±0.5) |
| Setelah reboot fuzzy OFF lagi | Status tidak persisten (memang sengaja) | Kirim `fuzzy on` lagi tiap boot |
| `fuzzy on` via Bluetooth → `ERR: Unknown command` | Bluetooth tidak meneruskan perintah fuzzy | Pakai USB Serial Monitor, bukan Bluetooth |

---

## 10. Referensi Kode

| File | Baris | Isi |
|---|---|---|
| `antarmuka.cpp` | 375-391 | Parser perintah `fuzzy` |
| `antarmuka.cpp` | 428 | Baris bantuan `help` |
| `antarmuka.cpp` | 489-491 | Status fuzzy di perintah `status` |
| `penggerak.cpp` | 274-282 | `setFuzzyEnabled` / `isFuzzyEnabled` |
| `penggerak.cpp` | 46-65 | Integrasi fuzzy ke PID (`hitung`) |
| `fuzzy_pid.cpp` | 98-143 | `compute()` inferensi + defuzzifikasi |
| `fuzzy_pid.cpp` | 145-153 | `setEnabled` / `isEnabled` / `reset` |
| `config.h` | 169-177 | `ENABLE_FUZZY_PID` & range fuzzy |
