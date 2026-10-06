// penggerak.h — PID line tracer + seluruh output motor servo.
#ifndef PENGGERAK_H
#define PENGGERAK_H

#include "ucode.h"
#include "config.h"

class Penggerak {
 public:
  void reset();  // PID + speed + telemetri kembali nol
  // Dipanggil saat tracer kembali dari recovery: buang hutang integral,
  // samakan lastError agar tak ada spike D di loop pertama.
  void resetIntegralSetelahRecovery(float errorBaru);
  // Hitung PID + terapkan ke motor. False = interval belum tiba,
  // pemanggil harus terapkanTerakhir().
  bool hitung(int bacaKiri, int bacaKanan, bool paksa = false);
  void terapkanTerakhir();  // pertahankan speed terakhir antar interval PID
  void jalankan(int speedKiri, int speedKanan);
  void berhenti();
  void saatBerhenti();  // berhenti + buang memori slew
  void majuManual(int speed, int durasi);
  void majuTerus();
  void belokKiri90(int ambang);
  void belokKanan90(int ambang);
  void putarKalibrasi();
  void putarCariGaris(float lastErr);
  float errorTerakhir() const;
  void telemetri(int bacaKiri, int bacaKanan);

 private:
  int terapkanDeadband(int speed);
  int batasiSlew(int target, int prev);

  float error = 0, lastError = 0;
  float P = 0, I = 0, D = 0, PID_value = 0;
  int prevKiri = 0, prevKanan = 0;
  int kecepatanKiri = 0, kecepatanKanan = 0;
  unsigned long pidTerakhir = 0;
  unsigned long telemTerakhir = 0;
  bool telemHeader = false;
};

#endif
