// fuzzy_pid.cpp — Implementasi Fuzzy adaptive PID controller.
// Inferensi Mamdani (min-AND) + defuzzifikasi weighted average (Sugeno-style
// singleton output). Tanpa alokasi dinamis: semua static const float.
//
// Masukan: error, delta_error (range ±FUZZY_ERROR_MAX / ±FUZZY_DERROR_MAX).
// Keluaran: delta_Kp (±FUZZY_DKP_MAX), delta_Kd (±FUZZY_DKD_MAX),
//           delta_Ki (±FUZZY_DKI_MAX).
//
// Tabel aturan simetris untuk line follower: respons simetris terhadap
// error positif/negatif. |error| besar → naikkan Kp; oscillasi (e≈0,
// |de| besar) → turunkan Kp, naikkan Kd.
#include "config.h"      // harus sebelum fuzzy_pid.h agar ENABLE_FUZZY_PID terdefinisi
#include "fuzzy_pid.h"

#if ENABLE_FUZZY_PID

// --- Tabel aturan: baris = error (NB..PB), kolom = deltaError (NB..PB) ---
// Nilai langsung berupa singleton output (bukan indeks MF).
// Simetris: rule[i][j] = rule[4-i][4-j] agar respons error+ dan error- identik.

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

// --- Fungsi keanggotaan segitiga ---
// NB: shoulder kiri  (a==b), PB: shoulder kanan (b==c), sisanya segitiga biasa.
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

// --- Fuzzifikasi: hitung 5 derajat keanggotaan untuk satu masukan ---
// MF: NB, NS, Z, PS, PB dengan range [-rangeMax, +rangeMax]
void FuzzyPID::fuzzify(float x, float rangeMax, float *out) {
  float h = rangeMax;        // batas range
  float q = rangeMax / 2.0;  // setengah range

  out[0] = trimf(x, -h, -h, -q);   // NB: shoulder kiri
  out[1] = trimf(x, -h, -q, 0.0);  // NS
  out[2] = trimf(x, -q, 0.0, q);   // Z
  out[3] = trimf(x, 0.0, q, h);    // PS
  out[4] = trimf(x, q, h, h);     // PB: shoulder kanan
}

// --- Defuzzifikasi weighted average (1D, untuk referensi) ---
float FuzzyPID::defuzzify(const float *fuzzOut, const float *range, int n) {
  float sumW = 0.0, sumWX = 0.0;
  for (int i = 0; i < n; i++) {
    sumW += fuzzOut[i];
    sumWX += fuzzOut[i] * range[i];
  }
  return (sumW > 0.0) ? sumWX / sumW : 0.0;
}

// --- Inisialisasi ---
void FuzzyPID::begin() {
  enabled = false;
}

// --- Compute: inferensi Mamdani + defuzzifikasi weighted average ---
// error dan deltaError di-clamp ke range valid sebelum fuzzifikasi.
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

void FuzzyPID::setEnabled(bool enable) {
  enabled = enable;
}

bool FuzzyPID::isEnabled() const {
  return enabled;
}

void FuzzyPID::reset() {
  enabled = false;
}

#endif  // ENABLE_FUZZY_PID
