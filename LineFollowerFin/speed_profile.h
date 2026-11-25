// speed_profile.h — Dynamic speed profile (Turbo mode) and adaptive speed logic.
#ifndef SPEED_PROFILE_H
#define SPEED_PROFILE_H

#include <Arduino.h>
#include "config.h"

class SpeedProfile {
public:
  SpeedProfile() = default;

  void begin();

  // Update speed profile based on PID error and output
  // Returns the computed base speed for this cycle
  int update(float error, float PID_value);

  // Reset turbo state (called on turn/recovery)
  void resetTurbo();

  // Getters
  int getBaseSpeed() const { return kecepatanDasar; }
  int getTurboActive() const { return kecepatanTurboAktif; }

  // Configuration (from globals)
  void setMaxSpeed(int maxSpeed) { kecepatanDasarMaks = maxSpeed; }
  void setMinSpeed(int minSpeed) { kecepatanDasarMin = minSpeed; }

private:
  // Dynamic speed state
  unsigned long waktuLurusMulai = 0;
  int kecepatanTurboAktif = 0;
  int kecepatanDasar = 0;
  int kecepatanDasarMaks = SPEED_TOMBOL_MAKS;
  int kecepatanDasarMin = SPEED_TOMBOL_MIN;
};

#endif // SPEED_PROFILE_H