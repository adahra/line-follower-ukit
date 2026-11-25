// motor_driver.h — Servo motor output with deadband and slew limiting.
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "config.h"

class MotorDriver {
public:
  MotorDriver() = default;

  void begin();

  // Set motor speeds directly (0-150, derated from 0-255)
  void run(int speedKiri, int speedKanan);

  // Like run(), but accepts signed speeds: positive=forward, negative=reverse, 0=stop.
  // Used by RC mode.
  void runSigned(int speedKiri, int speedKanan);

  void stop();
  void stopAndClear();  // stop + clear slew memory
  void forward(int speed);
  void turnLeft(int speed);
  void turnRight(int speed);
  void turnCalibration();
  void spinFindLine(float lastErr);

  // Apply last speeds (for PID interval gaps)
  void applyLast();

  int getLeftSpeed() const { return prevKiri; }
  int getRightSpeed() const { return prevKanan; }

  // Telemetry throttle
  void telemetry(int bacaKiri, int bacaKanan, float error, float PID_value, int kecepatanDasar);

private:
  int applyDeadband(int speed);
  int limitSlew(int target, int prev);

  int prevKiri = 0, prevKanan = 0;
  unsigned long telemTerakhir = 0;
  bool telemHeader = false;
};

#endif // MOTOR_DRIVER_H