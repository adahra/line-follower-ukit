// pid_controller.h — Pure PID computation logic.
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#if ENABLE_FUZZY_PID
#include "fuzzy_pid.h"
#endif
#if ENABLE_GAIN_SCHEDULING
// Gain scheduling multipliers defined in config.h
#endif

class PIDController {
public:
  PIDController() = default;

  void begin();

  // Compute PID output. Returns true if interval elapsed and output updated.
  bool compute(int bacaKiri, int bacaKanan, bool paksa = false);

  // Reset PID state (integral, lastError, timing)
  void reset();

  // Called when returning from recovery: clear integral windup, set lastError
  void resetAfterRecovery(float errorBaru);

  // Getters
  float getError() const { return error; }
  float getLastError() const { return lastError; }
  float getP() const { return P; }
  float getI() const { return I; }
  float getD() const { return D; }
  float getPIDValue() const { return PID_value; }

  // Gain access (use extern Kp, Ki, Kd from config.h)
  void setGains(float kp, float ki, float kd);

  // Fuzzy PID
#if ENABLE_FUZZY_PID
  void setFuzzyEnabled(bool on);
  bool isFuzzyEnabled() const;
  FuzzyPID &getFuzzyPID() { return fuzzyPid; }
#endif

  // Gain Scheduling
#if ENABLE_GAIN_SCHEDULING
  void setGainScheduling(bool on);
  bool isGainSchedulingEnabled() const;
  float getGainScheduleKp(int currentSpeed) const;
  float getGainScheduleKd(int currentSpeed) const;
  float getGainScheduleKi(int currentSpeed) const;
#endif

private:
  // PID State
  float error = 0, lastError = 0;
  float P = 0, I = 0, D = 0, PID_value = 0;
  unsigned long pidTerakhir = 0;

  // Fuzzy adaptive PID
#if ENABLE_FUZZY_PID
  FuzzyPID fuzzyPid;
#endif

  // Gain scheduling state
#if ENABLE_GAIN_SCHEDULING
  bool gainSchedulingEnabled = false;
#endif
};

#endif // PID_CONTROLLER_H