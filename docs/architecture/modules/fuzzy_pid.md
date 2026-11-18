# Modul FuzzyPID - Dokumentasi Teknis

## 1. Ikhtisar

FuzzyPID adalah fuzzy adaptive PID controller untuk LineFollowerFin. Ini menyesuaikan gain PID (Kp, Ki, Kd) berdasarkan error dan delta_error saat ini, memberikan kontrol yang dinamis dan responsif.

**File:** `LineFollowerFin/fuzzy_pid.h/.cpp` (1.3KB / 6.3KB)
**Dependencies:** `Arduino.h`, `config.h` (agar ENABLE_FUZZY_PID terdefinisi)

## 2. Desain & Arsitektur

### 2.1 Fuzzy Control
- **Input:** error (±FUZZY_ERROR_MAX), delta_error (±FUZZY_DERROR_MAX)
- **Output:** delta Kp (±FUZZY_DKP_MAX), delta Kd (±FUZZY_DKD_MAX), delta Ki (±FUZZY_DKI_MAX)
- **Inferensi:** Mamdani + defuzzifikasi centroid
- **No dynamic allocation:** Semua lookup table statis const

### 2.2 Arsitektur Fuzzy
```cpp
class FuzzyPID {
public:
  void begin();
  void compute(float error, float deltaError, float &dKp, float &dKd, float &dKi);
  void setEnabled(bool enable);
  bool isEnabled() const;
  void reset();

private:
  // Fungsi keanggotaan segitiga
  static float trimf(float x, float a, float b, float c);

  // Hitung 5 derajat keanggotaan untuk satu masukan
  void fuzzify(float x, float rangeMax, float *out);

  // Tabel aturan: baris = error (NB..PB), kolom = dError (NB..PB)
  static const float ruleDKp[5][5];
  static const float ruleDKd[5][5];
  static const float ruleDKi[5][5];

  // Defuzzifikasi centroid untuk satu output
  static float defuzzify(const float *fuzzOut, const float *range, int n);

  bool enabled = false;
};
```

## 3. Method & API

### 3.1 Method Publik

| Method | Signature | Keterangan |
|--------|-----------|-------------|
| `begin()` | `void` | Inisialisasi fuzzy PID |
| `compute()` | `void compute(float error, float deltaError, float &dKp, float &dKd, float &dKi)` | Hitung delta gains |
| `setEnabled()` | `void setEnabled(bool enable)` | Aktifkan/deaktifkan |
| `isEnabled()` | `bool isEnabled() const` | Cek apakah diaktifkan |
| `reset()` | `void` | Reset state |

### 3.2 Lookup Table
```cpp
static const float ruleDKp[5][5] = {
  {-5, -4, -3, -2, -1},  // error NB
  {-4, -3, -2, -1, -0.5}, // error NS
  {-3, -2, -1, -0.5, 0},  // error Z
  {-2, -1, -0.5, 0, 0.5}, // error PS
  {-1, -0.5, 0, 0.5, 1}   // error PB
};
// ruleDKd dan ruleDKi serupa
```

## 4. Integrasi dengan Modul Lain

### 4.1 Integrasi dengan Penggerak
- Bekerja sama untuk PID gains: `gerak.setFuzzyEnabled()`
- Bekerja sama untuk kontrol: fuzzy PID compute delta gains
- Bekerja sama untuk reset: `gerak.reset()`

### 4.2 Integrasi dengan SensorGaris
- Input: error dari `sensor.bacaKiri()` dan `sensor.bacaKanan()`
- Bekerja sama untuk delta_error (error terakhir)

## 5. Desain Decisions & Alasan

### 5.1 Fuzzy Control
**Alasan:** Fuzzy PID memberikan adaptasi gain yang halus dan intuitif. Tidak seperti relay feedback yang agresif, fuzzy control lebih gradual.

### 5.2 No Dynamic Allocation
**Alasan:** System embedded membutuhkan menghindari malloc/free. Lookup table statis aman.

### 5.3 Centroid Defuzzifikasi
**Alasan:** Centroid memberikan output yang smooth dan kontinu, cocok untuk kontrol motor.

## 6. Referensi Silang

- Lihat `LineFollowerFin.architecture.md` untuk arsitektur sistem
- Lihat `docs/api/fuzzy_pid.md` untuk dokumentasi API lengkap

