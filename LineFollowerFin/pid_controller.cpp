// pid_controller.cpp — PID computation implementation.
#include "pid_controller.h"
#include <string.h>

extern float Kp;
extern float Ki;
extern float Kd;

void PIDController::begin() {
  reset();
#if ENABLE_FUZZY_PID
  fuzzyPid.begin();
#endif
}

bool PIDController::compute(int bacaKiri, int bacaKanan, bool paksa) {
  unsigned long nowPid = millis();
  if (!paksa && nowPid - pidTerakhir < PID_INTERVAL_MS) {
    return false;
  }
  pidTerakhir = nowPid;

  // Compute error from current sensor readings (kiri=ID2, kanan=ID1)
  error = (bacaKiri - bacaKanan) / 2.0;

  P = error;
  I = I + error;
  I = constrain(I, -BATAS_INTEGRAL, BATAS_INTEGRAL);
  // raw delta error (not normalized) for fuzzy PID
  float deltaError = error - lastError;
  // dt same as PID interval for consistent D term
  const float dtPid = PID_INTERVAL_MS / 1000.0;
  D = deltaError / dtPid;

  // --- FUZZY ADAPTIVE PID ---
#if ENABLE_FUZZY_PID
  float KpEff = Kp, KdEff = Kd, KiEff = Ki;
  if (fuzzyPid.isEnabled()) {
    float dKp = 0.0, dKd = 0.0, dKi = 0.0;
    fuzzyPid.compute(error, deltaError, dKp, dKd, dKi);
    KpEff = Kp + dKp;
    KdEff = Kd + dKd;
    KiEff = Ki + dKi;
    // Clamp: effective gains must not be negative
    if (KpEff < 0.0) KpEff = 0.0;
    if (KdEff < 0.0) KdEff = 0.0;
    if (KiEff < 0.0) KiEff = 0.0;
  }
#else
  float KpEff = Kp, KdEff = Kd, KiEff = Ki;
#endif

  // --- GAIN SCHEDULING ---
#if ENABLE_GAIN_SCHEDULING
  if (gainSchedulingEnabled) {
    KpEff *= getGainScheduleKp();
    KdEff *= getGainScheduleKd();
    KiEff *= getGainScheduleKi();
  }
#endif

  PID_value = (KpEff * P) + (KiEff * I) + (KdEff * D);
  lastError = error;
  return true;
}

void PIDController::reset() {
  error = 0;
  lastError = 0;
  P = 0;
  I = 0;
  D = 0;
  PID_value = 0;
  pidTerakhir = 0;
}

void PIDController::resetAfterRecovery(float errorBaru) {
  I = 0;
  error = errorBaru;
  lastError = errorBaru;
}

void PIDController::setGains(float kp, float ki, float kd) {
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

#if ENABLE_FUZZY_PID
void PIDController::setFuzzyEnabled(bool on) {
  fuzzyPid.setEnabled(on);
}

bool PIDController::isFuzzyEnabled() const {
  return fuzzyPid.isEnabled();
}
#endif

#if ENABLE_GAIN_SCHEDULING
void PIDController::setGainScheduling(bool on) {
  gainSchedulingEnabled = on;
}

bool PIDController::isGainSchedulingEnabled() const {
  return gainSchedulingEnabled;
}

float PIDController::getGainScheduleKp(int currentSpeed) const {
  if (currentSpeed >= GAIN_SCHED_HIGH_SPEED) return GAIN_SCHED_KP_HIGH;
  if (currentSpeed <= GAIN_SCHED_LOW_SPEED) return GAIN_SCHED_KP_LOW;
  float t = (float)(currentSpeed - GAIN_SCHED_LOW_SPEED) /
            (GAIN_SCHED_HIGH_SPEED - GAIN_SCHED_LOW_SPEED);
  return GAIN_SCHED_KP_LOW + t * (GAIN_SCHED_KP_HIGH - GAIN_SCHED_KP_LOW);
}

float PIDController::getGainScheduleKd(int currentSpeed) const {
  if (currentSpeed >= GAIN_SCHED_HIGH_SPEED) return GAIN_SCHED_KD_HIGH;
  if (currentSpeed <= GAIN_SCHED_LOW_SPEED) return GAIN_SCHED_KD_LOW;
  float t = (float)(currentSpeed - GAIN_SCHED_LOW_SPEED) /
            (GAIN_SCHED_HIGH_SPEED - GAIN_SCHED_LOW_SPEED);
  return GAIN_SCHED_KD_LOW + t * (GAIN_SCHED_KD_HIGH - GAIN_SCHED_KD_LOW);
}

float PIDController::getGainScheduleKi(int currentSpeed) const {
  return GAIN_SCHED_KI;
}
#endif