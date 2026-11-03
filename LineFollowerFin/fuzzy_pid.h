// fuzzy_pid.h — Fuzzy adaptive PID controller.
// Masukan: error, delta_error (range ±10). Keluaran: delta Kp (±5), delta Kd (±2), delta Ki (±0.5).
// Inferensi Mamdani + defuzzifikasi centroid. Tanpa alokasi dinamis: semua static const float.
// Diaktifkan via ENABLE_FUZZY_PID di config.h.
#ifndef FUZZY_PID_H
#define FUZZY_PID_H

#include <Arduino.h>
#include "config.h"  // agar ENABLE_FUZZY_PID terdefinisi sebelum #if guard

#if ENABLE_FUZZY_PID

class FuzzyPID {
 public:
  FuzzyPID() = default;

  void begin();
  void compute(float error, float deltaError, float &dKp, float &dKd, float &dKi);
  void setEnabled(bool enable);
  bool isEnabled() const;
  void reset();

 private:
  // Fungsi keanggotaan segitiga: trimf(x, a, b, c)
  static float trimf(float x, float a, float b, float c);

  // Hitung 5 derajat keanggotaan untuk satu masukan (NB, NS, Z, PS, PB)
  void fuzzify(float x, float rangeMax, float *out);

  // Tabel aturan: baris = error (NB..PB), kolom = dError (NB..PB)
  static const float ruleDKp[5][5];
  static const float ruleDKd[5][5];
  static const float ruleDKi[5][5];

  // Defuzzifikasi centroid untuk satu output
  static float defuzzify(const float *fuzzOut, const float *range, int n);

  bool enabled = false;
};

#endif  // ENABLE_FUZZY_PID

#endif  // FUZZY_PID_H