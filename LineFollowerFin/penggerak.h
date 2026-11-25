// penggerak.h — PID line tracer + motor output facade.
// Composes PIDController, MotorDriver, SpeedProfile, HeadingTracker.
#ifndef PENGGERAK_H
#define PENGGERAK_H

#include "fwd.h"
#include "config.h"
#include "pid_controller.h"
#include "motor_driver.h"
#include "speed_profile.h"
#if ENABLE_HEADING_CORRECTION
#include "heading_tracker.h"
#endif

class Penggerak {
public:
  Penggerak();

  void reset();  // PID + speed + telemetry reset
  // Called when tracer returns from recovery: clear integral windup,
  // sync lastError to avoid D spike on first loop.
  void resetIntegralSetelahRecovery(float errorBaru);
  // Compute PID + apply to motors. False = interval not elapsed,
  // caller must call terapkanTerakhir().
  bool hitung(int bacaKiri, int bacaKanan, bool paksa = false);
  void terapkanTerakhir();  // maintain last speed between PID intervals
  void jalankan(int speedKiri, int speedKanan);
  // Like jalankan(), but accepts signed speeds: positive = forward,
  // negative = reverse, 0 = stop. Used by RC mode.
  void jalankanArah(int speedKiri, int speedKanan);
  void berhenti();
  void saatBerhenti();  // stop + clear slew memory
  void maju(int speed);
  void putarKiri(int speed);
  void putarKanan(int speed);
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
  void updateHeading();  // integrate gyro each loop (also when stopped/calibrating)
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

  // Access to sub-components for advanced use
  PIDController& getPIDController() { return pid; }
  MotorDriver& getMotorDriver() { return motor; }
  SpeedProfile& getSpeedProfile() { return speed; }
#if ENABLE_HEADING_CORRECTION
  HeadingTracker& getHeadingTracker() { return headingTracker; }
#endif

private:
  PIDController pid;
  MotorDriver motor;
  SpeedProfile speed;
#if ENABLE_HEADING_CORRECTION
  HeadingTracker headingTracker;
  float headingGain = HEADING_GAIN;  // runtime gain (can be loaded from EEPROM)
#endif

  // Forward heading correction to PID computation
  float applyHeadingCorrection(float pidValue);
};

#endif // PENGGERAK_H