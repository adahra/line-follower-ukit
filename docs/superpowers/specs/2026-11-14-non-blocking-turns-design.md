# Non-Blocking 90° Turns and Intersection State Machine

## Intent & Why
Projek robot line follower uKit memiliki manuver belok 90° dan penyeberangan persimpangan yang bersifat memblokir (`delay(300)` dan `while(...) delay(1)` hingga 1500 ms di `belokKiri90` / `belokKanan90`, serta `delay(150)` di `majuManual`). Selama proses ini, seluruh safety check (`aman.halangan()`, `aman.miring()`), tombol stop/darurat, telemetri, update Bluetooth, dan parser serial tidak berjalan sama sekali. Robot rentan menabrak halangan atau terjungkal saat belok.

Tujuan perubahan ini adalah memecah proses manuver persimpangan menjadi state machine non-blocking berbasis `millis()` di `loop()`.

## Goals
- Hilangkan semua pemanggilan `delay()` dan `while()` pada penanganan persimpangan dan belokan di runtime jalan robot.
- Pertahankan fungsi keselamatan (`aman.halangan()`, `aman.miring()`) dan UI responsif di setiap iterasi loop.
- Pertahankan kompatibilitas penuh dengan mode RACING (prediksi mapper) dan mode LEARNING (record event).
- Kode tetap lolos compile untuk target Mega2560 pada main sketch dan semua test sketch.

## Non-Goals
- Menambahkan sensor baru atau mengubah kontrol servo fisik.
- Mengubah algoritma PID normal saat tracking garis.

## Constraints
- Memori Mega2560 terbatas; jangan menambah alokasi dinamis (heap) atau library berat.
- Tetap ikuti konvensi kode (variabel bahasa Indonesia di modul inti).

## Approaches
1. **Approach B: Guard Polling di dalam loop blocking:**
   Mempertahankan fungsi linier `belokKiri90`, tetapi di dalam while disisipkan `if (aman.halangan()) return;`.
   *Trade-off:* Hanya menyelesaikan sebagian safety check, namun tetap memblokir update Bluetooth, telemetri, RC, dan parser serial. Ditolak.
2. **Approach A: State Machine di `loop()` (Dipilih):**
   Memperkenalkan substate pada `Status` (`SUBSTATE_TRACKING`, `SUBSTATE_INTER_CREEP`, `SUBSTATE_TURNING`). Manuver maju lewati persimpangan dan belok dipantau secara non-blocking dengan timer `millis()`.
   *Trade-off:* Sedikit perubahan pada `loop()` dan `Status`, tetapi menjamin loop utama bebas delay dan seluruh submodul tetap terpanggil tiap siklus.

## Design
1. **`status.h`:**
   Enum `SubState`:
   - `SUBSTATE_TRACKING = 0`: Mode line tracking normal / PID.
   - `SUBSTATE_INTER_CREEP = 1`: Robot melaju perlahan (150 ms) melewati garis persimpangan.
   - `SUBSTATE_TURNING = 2`: Robot memutar ke arah belokan sampai sensor menemukan garis atau timeout (1500 ms).
   Field baru di `struct Status`:
   - `SubState subState = SUBSTATE_TRACKING;`
   - `unsigned long subStateMulai = 0;`
   - `int targetAmbangBelok = 0;`
   - `ActionType subStateAksi = ACT_NONE;`

2. **`penggerak.h` / `penggerak.cpp`:**
   Tambah fungsi non-blocking untuk kendali langsung arah putaran:
   - `void putarKiri(int speed)`: motor kiri mundur (`dir=0`), motor kanan mundur (`dir=1`).
   - `void putarKanan(int speed)`: motor kiri maju (`dir=1`), motor kanan maju (`dir=0`).
   - `void maju(int speed)`: kedua motor maju.
   (Fungsi lama `belokKiri90`/`belokKanan90` dipertahankan untuk backward compatibility jika dibutuhkan test/skrip manual).

3. **`LineFollowerFin.ino` (`loop()`):**
   - Saat mendeteksi persimpangan (`bacaKiri >= ambangKiriEfektif && bacaKanan >= ambangKananEfektif`) dan `status.subState == SUBSTATE_TRACKING`:
     - Pindah ke `SUBSTATE_INTER_CREEP`.
     - `status.subStateMulai = millis()`.
     - Motor `gerak.maju(kecepatanDasarMin)`.
   - Di `SUBSTATE_INTER_CREEP`:
     - Cek apakah `millis() - status.subStateMulai >= 150`.
     - Jika tercapai: baca ulang sensor, tentukan aksi (`ACT_LEFT`, `ACT_RIGHT`, `ACT_STRAIGHT`) berdasarkan selisih atau prediksi RACING.
     - Jika `ACT_STRAIGHT`: kembali ke `SUBSTATE_TRACKING`, rekam event LEARNING jika aktif.
     - Jika `ACT_LEFT` / `ACT_RIGHT`: set `status.subStateAksi`, `status.targetAmbangBelok`, aktifkan `gerak.putarKiri` atau `gerak.putarKanan`, set `status.subState = SUBSTATE_TURNING`, `status.subStateMulai = millis()`.
   - Di `SUBSTATE_TURNING`:
     - Blind time: selama `millis() - status.subStateMulai < 250`, abaikan pembacaan sensor (agar tidak langsung trigger dari garis lama).
     - Setelah blind time: baca sensor target (`bacaKiri` untuk kiri, `bacaKanan` untuk kanan). Jika `>= targetAmbangBelok` ATAU `millis() - status.subStateMulai >= BELOK_TIMEOUT_MS`:
       - Selesai belok: `gerak.resetHeading()`, rekam event LEARNING, `status.subState = SUBSTATE_TRACKING`.
