// penggerak.h — PID line tracer + seluruh output motor servo.
#ifndef PENGGERAK_H
#define PENGGERAK_H

#include "fwd.h"
#include "config.h"
#if ENABLE_FUZZY_PID
#include "fuzzy_pid.h"
#endif
#if ENABLE_HEADING_CORRECTION
#include "heading_tracker.h"
#endif

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
  // Seperti jalankan(), tapi menerima kecepatan bertanda: positif = maju,
  // negatif = mundur, 0 = berhenti. Dipakai mode RC.
  void jalankanArah(int speedKiri, int speedKanan);
  void berhenti();
  void saatBerhenti();  // berhenti + buang memori slew
  void majuManual(int speed, int durasi);
  void majuTerus();
  void maju(int speed);
  void putarKiri(int speed);
  void putarKanan(int speed);
  void belokKiri90(int ambang);
  void belokKanan90(int ambang);
  void putarKalibrasi();
  void putarCariGaris(float lastErr);
  float errorTerakhir() const;
  void telemetri(int bacaKiri, int bacaKanan);

  // --- Fuzzy PID (adaptive gains) ---
#if ENABLE_FUZZY_PID
  void setFuzzyEnabled(bool on);
  bool isFuzzyEnabled() const;
#endif

  // --- Gain Scheduling ---
#if ENABLE_GAIN_SCHEDULING
  void setGainScheduling(bool on);
  bool isGainSchedulingEnabled() const;
  float getGainScheduleKp() const;
  float getGainScheduleKd() const;
  float getGainScheduleKi() const;
#endif

  // --- IMU Heading Correction ---
#if ENABLE_HEADING_CORRECTION
  void setHeadingCorrection(bool on);
  bool isHeadingCorrectionEnabled() const;
  void updateHeading();  // integrasi gyro tiap loop (juga saat berhenti/kalibrasi)
  void setHeadingTarget(float t);
  float getHeading() const;
  float getTargetHeading() const;
  float getHeadingError() const;
  void resetHeading();
  void calibrateHeadingBias();
  bool isHeadingCalibrating() const;
  void simpanHeading();
  void muatHeading();
#endif

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

  // Dynamic Speed State
  unsigned long waktuLurusMulai = 0;
  int kecepatanTurboAktif = 0;

  // Fuzzy adaptive PID (optional)
#if ENABLE_FUZZY_PID
  FuzzyPID fuzzyPid;
#endif

  // Gain scheduling state
#if ENABLE_GAIN_SCHEDULING
  bool gainSchedulingEnabled = false;
#endif

  // IMU heading correction state
#if ENABLE_HEADING_CORRECTION
  HeadingTracker headingTracker;
  float headingGain = HEADING_GAIN;  // gain runtime (bisa dimuat dari EEPROM)
#endif
};

#endif
