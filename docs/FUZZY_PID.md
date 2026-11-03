# Fuzzy Logic PID — Adaptive PID Line Follower UKIT

## 1. Pengertian

Fuzzy Logic PID (disebut juga *Fuzzy Adaptive PID*) adalah pengendali PID yang gain-nya (Kp, Kd, Ki) **tidak tetap**, tetapi disesuaikan secara dinamis oleh sebuah mesin inferensi fuzzy berdasarkan kondisi kesalahan (*error*) dan laju perubahan kesalahan (*delta error*) saat robot berjalan.

Pada PID statis, gain ditetapkan sekali (via tuning tombol atau EEPROM) dan berlaku untuk semua situasi: tikungan tajam, lurus, osilasi, maupun transient. Gain tunggal ini selalu kompromi — terlalu besar menyebabkan osilasi di lurus, terlalu kecil membuat robot lambat mengejar garis di tikungan.

Fuzzy adaptive PID mengatasi ini dengan menambahkan *delta gain* (ΔKp, ΔKd, ΔKi) yang dihitung dari error dan delta error:

```
Kp_eff = Kp + ΔKp
Kd_eff = Kd + ΔKd
Ki_eff = Ki + ΔKi
```

Sehingga pada tikungan tajam (|error| besar) Kp dinaikkan agar robot cepat mengejar garis, sedangkan saat osilasi (error ≈ 0, |delta error| besar) Kp diturunkan dan Kd dinaikkan agar robot lebih stabil. Ini meningkatkan performa PID statis tanpa perlu ganti gain manual.

Pada proyek ini, fuzzy PID bersifat **opsional** dan diaktifkan melalui `ENABLE_FUZZY_PID` di `config.h` (default `false` untuk hemat RAM ATMega2560).

---

## 2. Arsitektur

Fuzzy controller pada proyek ini memiliki:

- **2 masukan (input):**
  - `error` — kesalahan posisi garis (range ±`FUZZY_ERROR_MAX` = ±10)
  - `delta_error` — laju perubahan error (range ±`FUZZY_DERROR_MAX` = ±10)
- **3 keluaran (output):**
  - `delta_Kp` (range ±`FUZZY_DKP_MAX` = ±5)
  - `delta_Kd` (range ±`FUZZY_DKD_MAX` = ±2)
  - `delta_Ki` (range ±`FUZZY_DKI_MAX` = ±0.5)
- **Inferensi:** Mamdani (min-AND) — firing strength tiap aturan = `min(μ_error, μ_deltaError)`.
- **Defuzzifikasi:** Sugeno-style singleton weighted average (bukan centroid kontinu) — output aturan berupa nilai singleton, hasil akhir = `Σ(w·output) / Σw`.
- **Tanpa alokasi dinamis:** semua tabel aturan adalah `static const float` di flash.

File sumber:
- `LineFollowerFin/fuzzy_pid.h` — deklarasi class `FuzzyPID` (43 baris)
- `LineFollowerFin/fuzzy_pid.cpp` — implementasi (157 baris)
- Integrasi di `LineFollowerFin/penggerak.cpp` baris 46-65
- Konfigurasi di `LineFollowerFin/config.h` baris 169-177

---

## 3. Fungsi Keanggotaan (Membership Functions)

Setiap masukan difuzzifikasi menjadi 5 himpunan linguistik: **NB (Negative Big), NS (Negative Small), Z (Zero), PS (Positive Small), PB (Positive Big)**. Fungsi keanggotaan menggunakan segitiga (`trimf`) dengan shoulder handling untuk NB (shoulder kiri) dan PB (shoulder kanan).

```cpp
// fuzzy_pid.cpp:48-68
float FuzzyPID::trimf(float x, float a, float b, float c) {
  if (a == b) {
    // Shoulder kiri (NB): membership = 1 untuk x <= a, turun linear ke c
    if (x <= a) return 1.0;
    if (x >= c) return 0.0;
    return (c - x) / (c - a);
  }
  if (b == c) {
    // Shoulder kanan (PB): membership = 1 untuk x >= c, naik linear dari a
    if (x >= c) return 1.0;
    if (x <= a) return 0.0;
    return (x - a) / (c - a);
  }
  // Segitiga standar
  if (x <= a || x >= c) return 0.0;
  if (x == b) return 1.0;
  if (x < b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}
```

Fuzzifikasi menghitung 5 derajat keanggotaan untuk satu masukan dengan range `[-rangeMax, +rangeMax]`:

```cpp
// fuzzy_pid.cpp:70-81
void FuzzyPID::fuzzify(float x, float rangeMax, float *out) {
  float h = rangeMax;        // batas range
  float q = rangeMax / 2.0;  // setengah range

  out[0] = trimf(x, -h, -h, -q);   // NB: shoulder kiri
  out[1] = trimf(x, -h, -q, 0.0);  // NS
  out[2] = trimf(x, -q, 0.0, q);   // Z
  out[3] = trimf(x, 0.0, q, h);    // PS
  out[4] = trimf(x, q, h, h);     // PB: shoulder kanan
}
```

Dengan `rangeMax = 10`, titik-titik MF:
- NB: shoulder kiri, puncak di -10, turun ke -5
- NS: segitiga puncak -5, basis [-10, 0]
- Z: segitiga puncak 0, basis [-5, 5]
- PS: segitiga puncak +5, basis [0, 10]
- PB: shoulder kanan, puncak di +10, naik dari +5

---

## 4. Tabel Aturan (Rule Base)

Tabel aturan berukuran 5x5: baris = error (NB..PB), kolom = delta error (NB..PB). Nilai langsung berupa singleton output (bukan indeks MF). Tabel bersifat simetris: `rule[i][j] = rule[4-i][4-j]` agar respons error positif dan negatif identik.

```cpp
// fuzzy_pid.cpp:22-46
// delta_Kp: |error| besar → Kp naik; e≈0 + |de| besar → Kp turun (anti-oscillasi)
const float FuzzyPID::ruleDKp[5][5] = {
    {FUZZY_DKP_MAX,       FUZZY_DKP_MAX,       FUZZY_DKP_MAX,       FUZZY_DKP_MAX / 2.0, FUZZY_DKP_MAX / 2.0},  // e=NB
    {FUZZY_DKP_MAX,       FUZZY_DKP_MAX / 2.0, FUZZY_DKP_MAX / 2.0, FUZZY_DKP_MAX / 2.0, 0.0},                   // e=NS
    {-FUZZY_DKP_MAX / 2.0, 0.0,                 0.0,                 0.0,                -FUZZY_DKP_MAX / 2.0},      // e=Z
    {0.0,                  FUZZY_DKP_MAX / 2.0, FUZZY_DKP_MAX / 2.0, FUZZY_DKP_MAX / 2.0, FUZZY_DKP_MAX},          // e=PS
    {FUZZY_DKP_MAX / 2.0,  FUZZY_DKP_MAX / 2.0, FUZZY_DKP_MAX,       FUZZY_DKP_MAX,       FUZZY_DKP_MAX}           // e=PB
};

// delta_Kd: |de| besar → Kd naik (damping); e besar + de kecil → Kd turun
const float FuzzyPID::ruleDKd[5][5] = {
    {FUZZY_DKD_MAX / 2.0, -FUZZY_DKD_MAX / 2.0, -FUZZY_DKD_MAX,    -FUZZY_DKD_MAX / 2.0, FUZZY_DKD_MAX / 2.0},  // e=NB
    {FUZZY_DKD_MAX / 2.0,  0.0,                 -FUZZY_DKD_MAX / 2.0, 0.0,                FUZZY_DKD_MAX / 2.0},  // e=NS
    {FUZZY_DKD_MAX,        FUZZY_DKD_MAX / 2.0,   0.0,                FUZZY_DKD_MAX / 2.0, FUZZY_DKD_MAX},        // e=Z
    {FUZZY_DKD_MAX / 2.0,  0.0,                 -FUZZY_DKD_MAX / 2.0, 0.0,                FUZZY_DKD_MAX / 2.0},  // e=PS
    {FUZZY_DKD_MAX / 2.0, -FUZZY_DKD_MAX / 2.0, -FUZZY_DKD_MAX,    -FUZZY_DKD_MAX / 2.0, FUZZY_DKD_MAX / 2.0}   // e=PB
};

// delta_Ki: error persisten (e besar, de≈0) → Ki naik; oscillasi → Ki turun
const float FuzzyPID::ruleDKi[5][5] = {
    {0.0,                 FUZZY_DKI_MAX / 2.0, FUZZY_DKI_MAX,       FUZZY_DKI_MAX / 2.0, 0.0},                    // e=NB
    {0.0,                 0.0,                 FUZZY_DKI_MAX / 2.0, 0.0,                 0.0},                    // e=NS
    {-FUZZY_DKI_MAX / 2.0, 0.0,                 0.0,                 0.0,                -FUZZY_DKI_MAX / 2.0},     // e=Z
    {0.0,                 0.0,                 FUZZY_DKI_MAX / 2.0, 0.0,                 0.0},                    // e=PS
    {0.0,                 FUZZY_DKI_MAX / 2.0, FUZZY_DKI_MAX,       FUZZY_DKI_MAX / 2.0, 0.0}                     // e=PB
};
```

### Semantik aturan (penjelasan)

- **ruleDKp:** Saat |error| besar (robot jauh dari garis), Kp dinaikkan agar respons cepat. Saat error ≈ 0 tetapi |delta error| besar (robot berosilasi di sekitar garis), Kp diturunkan untuk meredam osilasi.
- **ruleDKd:** Saat |delta error| besar (perubahan cepat), Kd dinaikkan untuk damping. Saat error besar tapi delta error kecil (robot mendekati garis dengan stabil), Kd diturunkan agar tidak terlalu lambat.
- **ruleDKi:** Saat error persisten (error besar, delta error ≈ 0 — robot stuck di offset), Ki dinaikkan untuk menghilangkan steady-state error. Saat osilasi (error ≈ 0, delta error besar), Ki diturunkan agar integral tidak menumpuk.

---

## 5. Alur Inferensi (compute)

Metode `compute()` melakukan: clamp masukan, fuzzify kedua masukan, evaluasi 25 aturan dengan min-AND, lalu weighted average defuzzification.

```cpp
// fuzzy_pid.cpp:98-143
void FuzzyPID::compute(float error, float deltaError,
                       float &dKp, float &dKd, float &dKi) {
  dKp = 0.0;
  dKd = 0.0;
  dKi = 0.0;

  if (!enabled) return;

  // Clamp masukan ke range fuzzifikasi
  if (error > FUZZY_ERROR_MAX) error = FUZZY_ERROR_MAX;
  if (error < -FUZZY_ERROR_MAX) error = -FUZZY_ERROR_MAX;
  if (deltaError > FUZZY_DERROR_MAX) deltaError = FUZZY_DERROR_MAX;
  if (deltaError < -FUZZY_DERROR_MAX) deltaError = -FUZZY_DERROR_MAX;

  // Fuzzifikasi kedua masukan
  float muE[5];
  float muDE[5];
  fuzzify(error, FUZZY_ERROR_MAX, muE);
  fuzzify(deltaError, FUZZY_DERROR_MAX, muDE);

  // Evaluasi aturan: firing strength = min(muE[i], muDE[j]) (Mamdani AND)
  // Defuzzifikasi: weighted average dari singleton output
  float sumW = 0.0;
  float sumWKp = 0.0, sumWKd = 0.0, sumWKi = 0.0;

  for (int i = 0; i < 5; i++) {
    if (muE[i] <= 0.0) continue;  // skip: aturan dengan firing strength 0
    for (int j = 0; j < 5; j++) {
      float w = (muE[i] < muDE[j]) ? muE[i] : muDE[j];  // min()
      if (w <= 0.0) continue;

      sumW += w;
      sumWKp += w * ruleDKp[i][j];
      sumWKd += w * ruleDKd[i][j];
      sumWKi += w * ruleDKi[i][j];
    }
  }

  if (sumW > 0.0) {
    dKp = sumWKp / sumW;
    dKd = sumWKd / sumW;
    dKi = sumWKi / sumW;
  }
}
```

### Langkah-langkah inferensi

1. **Cek enabled** — jika `false`, output tetap 0 (gain tidak berubah).
2. **Clamp input** — error ke ±`FUZZY_ERROR_MAX`, deltaError ke ±`FUZZY_DERROR_MAX`.
3. **Fuzzify** — hitung `muE[5]` dan `muDE[5]`.
4. **Evaluasi 25 aturan** — untuk setiap kombinasi (i,j), firing strength `w = min(muE[i], muDE[j])`. Skip jika w ≤ 0.
5. **Akumulasi** — `sumW += w`, `sumWKp += w * ruleDKp[i][j]`, dst.
6. **Defuzzify** — `dKp = sumWKp / sumW` (dan serupa untuk Kd, Ki). Jika `sumW == 0`, output tetap 0.

---

## 6. Integrasi dengan PID

Pada `Penggerak::hitung()`, fuzzy menghasilkan gain efektif yang digunakan untuk menghitung `PID_value`:

```cpp
// penggerak.cpp:46-65
// --- FUZZY ADAPTIVE PID ---
// Jika fuzzy aktif, hitung delta Kp/Kd/Ki dari error & delta_error,
// lalu gunakan gain efektif (base + delta) untuk PID.
#if ENABLE_FUZZY_PID
  float KpEff = Kp, KdEff = Kd, KiEff = Ki;
  if (fuzzyPid.isEnabled()) {
    float dKp = 0.0, dKd = 0.0, dKi = 0.0;
    fuzzyPid.compute(error, D, dKp, dKd, dKi);  // D = delta error
    KpEff = Kp + dKp;
    KdEff = Kd + dKd;
    KiEff = Ki + dKi;
    // Clamp: gain efektif tidak boleh negatif
    if (KpEff < 0.0) KpEff = 0.0;
    if (KdEff < 0.0) KdEff = 0.0;
    if (KiEff < 0.0) KiEff = 0.0;
  }
  PID_value = (KpEff * P) + (KiEff * I) + (KdEff * D);
#else
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
#endif
```

Catatan: `D` (delta error) yang diteruskan ke fuzzy adalah `error - lastError` dari loop PID yang sama, sehingga konsisten.

`Penggerak` menyediakan wrapper untuk mengaktifkan/nonaktifkan fuzzy (penggerak.cpp:274-282):

```cpp
#if ENABLE_FUZZY_PID
void Penggerak::setFuzzyEnabled(bool on) {
  fuzzyPid.setEnabled(on);
}

bool Penggerak::isFuzzyEnabled() const {
  return fuzzyPid.isEnabled();
}
#endif
```

---

## 7. Konfigurasi

Di `config.h` baris 169-177:

```cpp
// --- FUZZY PID ---
#define ENABLE_FUZZY_PID false         // aktifkan fuzzy adaptive PID
#define FUZZY_ERROR_MAX 10.0           // range error masukan [-10, 10]
#define FUZZY_DERROR_MAX 10.0          // range delta_error masukan [-10, 10]
#define FUZZY_DKP_MAX 5.0              // range delta Kp keluaran [-5, 5]
#define FUZZY_DKD_MAX 2.0              // range delta Kd keluaran [-2, 2]
#define FUZZY_DKI_MAX 0.5              // range delta Ki keluaran [-0.5, 0.5]
```

| Konstanta | Nilai | Keterangan |
|---|---|---|
| `ENABLE_FUZZY_PID` | `false` | Master switch compile-time |
| `FUZZY_ERROR_MAX` | 10.0 | Range input error ±10 |
| `FUZZY_DERROR_MAX` | 10.0 | Range input delta error ±10 |
| `FUZZY_DKP_MAX` | 5.0 | Range output delta Kp ±5 |
| `FUZZY_DKD_MAX` | 2.0 | Range output delta Kd ±2 |
| `FUZZY_DKI_MAX` | 0.5 | Range output delta Ki ±0.5 |

---

## 8. Enable/Disable Flow

### Compile-time
- `ENABLE_FUZZY_PID = false` (default): class `FuzzyPID` tidak dikompilasi (guard `#if`), `penggerak` pakai PID statis. Tidak ada overhead RAM.
- `ENABLE_FUZZY_PID = true`: fuzzy tersedia, tetapi default runtime `enabled = false` (lihat `begin()`).

### Runtime (serial command)
Jika Bluetooth/serial aktif, perintah serial:
```
fuzzy on      → gerak.setFuzzyEnabled(true)   → "Fuzzy PID: ON (adaptive gains aktif)"
fuzzy off     → gerak.setFuzzyEnabled(false)  → "Fuzzy PID: OFF (static gains dari config/EEPROM)"
fuzzy         → tampilkan status ON/OFF
```

Kode di `antarmuka.cpp:375-391`:

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
```

Bantuan perintah di `antarmuka.cpp:428`: `fuzzy on|off : aktifkan/nonaktifkan/cek Fuzzy adaptive PID`. Status juga ditampilkan di `antarmuka.cpp:489-491` (`| Fuzzy: ON/OFF`). Tidak ada tombol fisik untuk fuzzy — hanya serial.

---

## 9. Kasus Penggunaan

Fuzzy bermanfaat pada situasi:
- **Tikungan tajam:** |error| besar → Kp naik → robot cepat mengejar garis.
- **Osilasi di lurus:** error ≈ 0, |delta error| besar → Kp turun, Kd naik → stabil.
- **Transient (perubahan arah mendadak):** Kd naik → damping.
- **Steady-state offset:** Ki naik → hilangkan error persisten.

### Contoh numerik
Misal Kp=2.0, Kd=1.0, Ki=0.1 (base). Error = 8, delta error = 1 (robot menyimpang jauh, perubahan kecil).
- Fuzzifikasi: muE[PB] tinggi, muDE[PS] tinggi.
- Aturan yang fire: ruleDKp[PB][PS] = FUZZY_DKP_MAX = 5.0 → dKp ≈ 5.0.
- Kp_eff ≈ 2.0 + 5.0 = 7.0 → respons agresif.

Misal error = 0.5, delta error = 8 (robot berosilasi).
- ruleDKp[Z][PB] = -FUZZY_DKP_MAX/2 = -2.5 → Kp_eff ≈ 2.0 - 2.5 = 0 (clamp ≥0 → 0? clamp ke 0) → respons diredam.
- ruleDKd[Z][PB] = FUZZY_DKD_MAX = 2.0 → Kd_eff ≈ 1.0 + 2.0 = 3.0 → damping kuat.

---

## 10. Keterbatasan & Gotchas

1. **Tabel aturan simetris** — respons error+ dan error- identik; cocok line follower simetris, tetapi tidak dapat menangani asimetri mekanis (misal motor kiri/kanan berbeda).
2. **Tanpa optimasi runtime** — 25 aturan dievaluasi setiap loop; pada ATMega2560 dengan interval 10ms masih OK, tetapi tambahan beban CPU.
3. **Clamp input** — error/deltaError di-clamp ke ±range sebelum fuzzifikasi; nilai ekstrem kehilangan informasi gradien.
4. **Default OFF** — `ENABLE_FUZZY_PID = false` dan runtime `enabled = false`; harus diaktifkan manual via serial.
5. **RAM overhead minimal** — tabel aturan `static const float` di flash (3×25 float = 75 float ≈ 300 byte flash), objek `FuzzyPID` hanya satu bool `enabled`.
6. **Gain efektif tidak persisten** — ΔKp/Kd/Ki tidak disimpan ke EEPROM; hanya base Kp/Kd/Ki yang disimpan.
7. **`defuzzify()` 1D tidak digunakan** — metode `defuzzify(const float*, const float*, int)` ada di header tetapi `compute()` melakukan weighted average langsung; jangan bingung.
8. **Fuzzy tidak aktif saat disabled** — `compute()` langsung return 0 tanpa mengubah gain; PID pakai gain statis.
9. **Tidak ada tombol fisik** — hanya serial command; jika serial tidak aktif, fuzzy tidak bisa di-toggle runtime.

---

## 11. Perbandingan dengan PID Statis

| Aspek | PID Statis | Fuzzy Adaptive PID |
|---|---|---|
| Gain | Tetap (Kp, Kd, Ki) | Base + Δ (adaptif) |
| Tuning | Manual (tombol/EEPROM) | Otomatis berdasarkan error & delta error |
| Respons tikungan | Kompromi | Agresif saat |error| besar |
| Stabilitas lurus | Kompromi | Diredam saat osilasi |
| Overhead CPU | Rendah | +25 aturan/loop |
| RAM | Minimal | Minimal (+1 bool) |
| Konfigurasi | `ENABLE_FUZZY_PID=false` | `ENABLE_FUZZY_PID=true` + serial `fuzzy on` |
| Persistensi gain | EEPROM | Base di EEPROM, Δ tidak persisten |

---

## 12. Rekomendasi Pengembangan

1. **Aktifkan fuzzy di config.h** (`ENABLE_FUZZY_PID = true`) dan uji di track nyata sebelum deploy.
2. **Tuning base gain** — fuzzy bekerja sebagai delta di atas base; base Kp/Kd/Ki harus sudah wajar (tuning manual dulu).
3. **Tambahkan tombol fisik** untuk toggle fuzzy tanpa serial (opsional).
4. **Simetris → asimetris** — jika motor kiri/kanan berbeda, buat tabel aturan non-simetris atau tambah masukan ketiga (misal error motor).
5. **Optimasi** — precompute atau kurangi aturan (misal 3x3) untuk hemat CPU jika perlu.
6. **Persistensi status fuzzy** — simpan flag enabled ke EEPROM agar tidak perlu serial setiap boot.
7. **Logging telemetri** — cetak KpEff/KdEff/KiEff ke serial untuk analisis performa fuzzy.
8. **Validasi dengan data** — rekam error/delta error dan gain efektif, bandingkan dengan PID statis di track yang sama.

---

## 13. Alur Eksekusi di loop() Utama

Fuzzy PID tidak dipanggil langsung di `loop()`. Alurnya:

1. `loop()` memanggil `gerak.hitung(bacaKiri, bacaKanan)` pada interval `PID_INTERVAL_MS` (10ms).
2. Di dalam `hitung()`, error dan delta error dihitung.
3. Jika `ENABLE_FUZZY_PID` aktif dan `fuzzyPid.isEnabled()`, `compute()` dipanggil untuk menghasilkan dKp/dKd/dKi.
4. Gain efektif di-clamp (≥0) lalu digunakan untuk `PID_value`.
5. `PID_value` diterapkan ke motor via `jalankan()`.

Jika fuzzy disabled (default), jalur `#else` digunakan: `PID_value = (Kp * P) + (Ki * I) + (Kd * D)` tanpa overhead inferensi.

---

## 14. Detail Implementasi trimf (Edge Cases)

- **Shoulder kiri (NB):** `a == b` → membership 1 untuk `x <= a`, turun linear ke 0 di `c`. Memastikan nilai ekstrem negatif tetap fully member.
- **Shoulder kanan (PB):** `b == c` → membership 1 untuk `x >= c`, naik linear dari `a`. Memastikan nilai ekstrem positif tetap fully member.
- **Sigitiga standar:** membership 0 di luar [a,c], puncak 1 di b. Pembagian oleh (b-a) dan (c-b) aman karena a<b<c selalu (kecuali shoulder).
- **Pembagian nol:** tidak terjadi karena konfigurasi MF selalu a < b < c untuk segitiga, dan shoulder case ditangani terpisah.

---

## 15. Cara Mengaktifkan & Menguji

1. Edit `config.h`: `#define ENABLE_FUZZY_PID true`
2. Compile: `arduino-cli compile --fqbn arduino:avr:mega --build-path /tmp/avrbuild LineFollowerFin.ino`
3. Upload ke robot.
4. Hubungkan serial (Bluetooth/USB), kirim `fuzzy on`.
5. Jalankan robot (`start`), amati respons di tikungan vs lurus.
6. Bandingkan dengan `fuzzy off` untuk melihat perbedaan.
7. Tuning base gain (Kp/Kd/Ki) terlebih dahulu agar fuzzy bekerja di atas base yang wajar.

---

## 16. FAQ

**Q: Apakah fuzzy menggantikan tuning manual?**
A: Tidak. Fuzzy bekerja sebagai delta di atas base gain. Base gain harus dituning dulu.

**Q: Kenapa default OFF?**
A: Untuk hemat RAM/CPU pada ATMega2560, dan untuk memudahkan debugging PID statis terlebih dahulu.

**Q: Bisakah fuzzy disimpan ke EEPROM?**
A: Saat ini tidak. Status enabled hanya runtime (RAM). Perlu modifikasi untuk persistensi.

**Q: Apa bedanya defuzzify() 1D dengan compute()?**
A: `defuzzify()` adalah helper centroid 1D yang tidak digunakan; `compute()` langsung melakukan weighted average dari 3 tabel sekaligus (lebih efisien).

---

## 17. Referensi Kode

| File | Baris | Isi |
|---|---|---|
| `fuzzy_pid.h` | 1-43 | Deklarasi class FuzzyPID |
| `fuzzy_pid.cpp` | 1-157 | Implementasi lengkap |
| `penggerak.h` | 7-9, 34-37, 56-58 | Include & member fuzzyPid |
| `penggerak.cpp` | 46-65 | Integrasi fuzzy ke PID |
| `penggerak.cpp` | 274-282 | setFuzzyEnabled/isFuzzyEnabled |
| `config.h` | 169-177 | Konfigurasi fuzzy |
| `antarmuka.cpp` | 375-391 | Serial command fuzzy |
| `antarmuka.cpp` | 428 | Bantuan perintah |
| `antarmuka.cpp` | 489-491 | Status fuzzy di serial |
