// motor_driver.cpp — Motor output implementation.
#include "motor_driver.h"
#include "ukit_wrapper.h"  // setServo, readInfraredDistance
#include <string.h>

void MotorDriver::begin() {
  // Initialize servos to neutral
  run(0, 0);
}

void MotorDriver::run(int speedKiri, int speedKanan) {
  // Clamp to servo limits (derated 0-150)
  speedKiri = constrain(speedKiri, 0, SERVO_MAKS_DIPAKAI);
  speedKanan = constrain(speedKanan, 0, SERVO_MAKS_DIPAKAI);

  // Deadband compensation
  speedKiri = applyDeadband(speedKiri);
  speedKanan = applyDeadband(speedKanan);

  // Slew rate limiting
  speedKiri = limitSlew(speedKiri, prevKiri);
  speedKanan = limitSlew(speedKanan, prevKanan);
  prevKiri = speedKiri;
  prevKanan = speedKanan;

  // Apply to servos: kiri = servos 4+2 (dir 1), kanan = servos 1+3 (dir 0)
  // setServoTurn(id, dir, speed)
  setServoTurn(4, 1, speedKiri);
  setServoTurn(2, 1, speedKiri);
  setServoTurn(1, 0, speedKanan);
  setServoTurn(3, 0, speedKanan);
}

void MotorDriver::runSigned(int speedKiri, int speedKanan) {
  // Convert signed to unsigned with direction
  // Positive = forward, negative = reverse
  // For now, just clamp and use run()
  // Note: uKit servos don't support reverse easily, so we clamp to 0
  if (speedKiri < 0) speedKiri = 0;
  if (speedKanan < 0) speedKanan = 0;
  run(speedKiri, speedKanan);
}

void MotorDriver::stop() {
  run(0, 0);
}

void MotorDriver::stopAndClear() {
  stop();
  prevKiri = 0;
  prevKanan = 0;
}

void MotorDriver::forward(int speed) {
  run(speed, speed);
}

void MotorDriver::turnLeft(int speed) {
  run(speed, speed);
  // Actually for differential drive turn left: left wheel slower/back, right faster
  // But uKit uses continuous rotation servos, so we need to handle differently
  // Original implementation: both forward at same speed for pivot? No, let's check
  // The original putarKiri uses jalankan(speed, speed) which runs both forward
  // This might be a pivot turn using speed difference... but both same speed?
  // Let me check original: it calls jalankan(speed, speed) - that's both motors same speed
  // That would go straight, not turn. But maybe the robot turns by differential?
  // Wait, the original code for putarKiri calls jalankan(speed, speed) which calls
  // setServo for both sides at same speed. That's forward.
  // Actually looking at the original code more carefully:
  // putarKiri -> jalankan(speed, speed) -> setServo(4, speed, 1), setServo(2, speed, 1) for left
  // and setServo(1, speed, 0), setServo(3, speed, 0) for right
  // Both sides same speed = forward. But the name says "putarKiri" (turn left).
  // This seems like a bug or the robot turns by other means.
  // For now, replicate original behavior:
  run(speed, speed);
}

void MotorDriver::turnRight(int speed) {
  run(speed, speed);
}

void MotorDriver::turnCalibration() {
  // Original: putarKalibrasi() -> jalankan(120, 120)
  run(120, 120);
}

void MotorDriver::spinFindLine(float lastErr) {
  // Original: putarCariGaris - spins toward last error direction
  int speed = 100;
  if (lastErr > 0) {
    // Error positive = right sensor sees more = turn right to find line
    run(0, speed);  // Right motor forward, left stopped
  } else {
    run(speed, 0);  // Left motor forward, right stopped
  }
}

void MotorDriver::applyLast() {
  run(prevKiri, prevKanan);
}

int MotorDriver::applyDeadband(int speed) {
  if (speed > 0 && speed < SERVO_MIN_EFEKTIF) {
    return SERVO_MIN_EFEKTIF;
  }
  return speed;
}

int MotorDriver::limitSlew(int target, int prev) {
  int diff = target - prev;
  if (diff > SLEW_MAKS_PER_LOOP) return prev + SLEW_MAKS_PER_LOOP;
  if (diff < -SLEW_MAKS_PER_LOOP) return prev - SLEW_MAKS_PER_LOOP;
  return target;
}

void MotorDriver::telemetry(int bacaKiri, int bacaKanan, float error, float PID_value, int kecepatanDasar) {
  if (millis() - telemTerakhir < 100) return;
  telemTerakhir = millis();

  if (!telemHeader) {
    Serial.println(F("t,err,pid,dasar,kiri,kanan,senK,senN"));
    telemHeader = true;
  }
  Serial.print(millis());
  Serial.print(F(","));
  Serial.print(error, 2);
  Serial.print(F(","));
  Serial.print(PID_value, 2);
  Serial.print(F(","));
  Serial.print(kecepatanDasar);
  Serial.print(F(","));
  Serial.print(prevKiri);
  Serial.print(F(","));
  Serial.print(prevKanan);
  Serial.print(F(","));
  Serial.print(bacaKiri);
  Serial.print(F(","));
  Serial.println(bacaKanan);
}