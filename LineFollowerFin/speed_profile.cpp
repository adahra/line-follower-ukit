// speed_profile.cpp — Dynamic speed profile implementation.
#include "speed_profile.h"

void SpeedProfile::begin() {
  resetTurbo();
  kecepatanDasar = kecepatanDasarMin;
}

int SpeedProfile::update(float error, float PID_value) {
  // --- DYNAMIC SPEED PROFILE (Turbo Mode) ---
  // If error small (straight) for long time, increase base speed.
  if (abs(error) < LURUS_THRESHOLD) {
    if (waktuLurusMulai == 0) {
      waktuLurusMulai = millis();
    } else if (millis() - waktuLurusMulai > LURUS_TIME_REQUIRED) {
      // Enter Turbo mode: gradually increase speed to avoid slip
      kecepatanTurboAktif += TURBO_RAMP_SPEED;
      if (kecepatanTurboAktif > KECEPATAN_TURBO) {
        kecepatanTurboAktif = KECEPATAN_TURBO;
      }
    }
  } else {
    // Turn detected: reset turbo immediately
    waktuLurusMulai = 0;
    kecepatanTurboAktif = 0;
  }

  // 3. Adaptive base speed: more deviation = more braking.
  int faktorPengurang = abs(PID_value) * FAKTOR_REM_ADAPTIF;

  // Base speed = max speed + turbo bonus - adaptive braking
  int baseSpeedTarget = kecepatanDasarMaks + kecepatanTurboAktif - faktorPengurang;
  kecepatanDasar = constrain(baseSpeedTarget, kecepatanDasarMin, SERVO_MAKS_DIPAKAI);

  return kecepatanDasar;
}

void SpeedProfile::resetTurbo() {
  waktuLurusMulai = 0;
  kecepatanTurboAktif = 0;
}