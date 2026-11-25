// penggerak.cpp — Facade implementation using PIDController, MotorDriver, SpeedProfile.
#include "penggerak.h"
#include <string.h>
#include "eeprom_layout.h"

Penggerak::Penggerak() {
  // Sub-components initialize in their begin() called from reset()
}

void Penggerak::reset() {
  pid.begin();
  motor.begin();
  speed.begin();
#if ENABLE_HEADING_CORRECTION
  headingTracker.begin();
  headingTracker.setEnabled(false);
  headingGain = HEADING_GAIN;
#endif
}

void Penggerak::resetIntegralSetelahRecovery(float errorBaru) {
  pid.resetAfterRecovery(errorBaru);
#if ENABLE_HEADING_CORRECTION
  if (headingTracker.isEnabled()) {
    headingTracker.reset();
  }
#endif
}

bool Penggerak::hitung(int bacaKiri, int bacaKanan, bool paksa) {
  // Compute PID (includes fuzzy, gain scheduling)
  bool pidUpdated = pid.compute(bacaKiri, bacaKanan, paksa);
  if (!pidUpdated) {
    return false;
  }

  // Apply heading correction to PID value
  float pidValue = applyHeadingCorrection(pid.getPIDValue());

  // Update speed profile
  int baseSpeed = speed.update(pid.getError(), pidValue);

  // Compute motor speeds
  int speedKiri = baseSpeed - pidValue;
  int speedKanan = baseSpeed + pidValue;

  // Apply to motors (includes deadband, slew, clamping)
  motor.run(speedKiri, speedKanan);

  // Telemetry
  motor.telemetry(bacaKiri, bacaKanan, pid.getError(), pidValue, baseSpeed);

  return true;
}

void Penggerak::terapkanTerakhir() {
  motor.applyLast();
}

void Penggerak::jalankan(int speedKiri, int speedKanan) {
  motor.run(speedKiri, speedKanan);
}

void Penggerak::jalankanArah(int speedKiri, int speedKanan) {
  motor.runSigned(speedKiri, speedKanan);
}

void Penggerak::berhenti() {
  motor.stop();
}

void Penggerak::saatBerhenti() {
  motor.stopAndClear();
}

void Penggerak::maju(int speed) {
  motor.forward(speed);
}

void Penggerak::putarKiri(int speed) {
  motor.turnLeft(speed);
}

void Penggerak::putarKanan(int speed) {
  motor.turnRight(speed);
}

void Penggerak::putarKalibrasi() {
  motor.turnCalibration();
}

void Penggerak::putarCariGaris(float lastErr) {
  motor.spinFindLine(lastErr);
}

float Penggerak::errorTerakhir() const {
  return pid.getLastError();
}

void Penggerak::telemetri(int bacaKiri, int bacaKanan) {
  // Use motor's telemetry with current PID values
  motor.telemetry(bacaKiri, bacaKanan, pid.getError(), pid.getPIDValue(), speed.getBaseSpeed());
}

// --- Fuzzy PID ---
#if ENABLE_FUZZY_PID
void Penggerak::setFuzzyEnabled(bool on) {
  pid.setFuzzyEnabled(on);
}

bool Penggerak::isFuzzyEnabled() const {
  return pid.isFuzzyEnabled();
}
#endif

// --- Gain Scheduling ---
#if ENABLE_GAIN_SCHEDULING
void Penggerak::setGainScheduling(bool on) {
  pid.setGainScheduling(on);
}

bool Penggerak::isGainSchedulingEnabled() const {
  return pid.isGainSchedulingEnabled();
}

float Penggerak::getGainScheduleKp() const {
  return pid.getGainScheduleKp(speed.getBaseSpeed());
}

float Penggerak::getGainScheduleKd() const {
  return pid.getGainScheduleKd(speed.getBaseSpeed());
}

float Penggerak::getGainScheduleKi() const {
  return pid.getGainScheduleKi(speed.getBaseSpeed());
}
#endif

// --- IMU Heading Correction ---
#if ENABLE_HEADING_CORRECTION
void Penggerak::setHeadingCorrection(bool on) {
  headingTracker.setEnabled(on);
}

bool Penggerak::isHeadingCorrectionEnabled() const {
  return headingTracker.isEnabled();
}

void Penggerak::updateHeading() {
  headingTracker.update();
}

void Penggerak::setHeadingTarget(float t) {
  headingTracker.setTarget(t);
}

float Penggerak::getHeading() const {
  return headingTracker.getHeading();
}

float Penggerak::getTargetHeading() const {
  return headingTracker.getTargetHeading();
}

float Penggerak::getHeadingError() const {
  return headingTracker.getHeadingError();
}

void Penggerak::resetHeading() {
  headingTracker.reset();
}

void Penggerak::calibrateHeadingBias() {
  headingTracker.calibrateBias();
}

bool Penggerak::isHeadingCalibrating() const {
  return headingTracker.isCalibrating();
}

void Penggerak::simpanHeading() {
  EEPROM.update(EEPROM_ADDR::EE_ADDR_HEADING_MAGIC, MAGIC_HEADING_VALID);
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_HEADING_GAIN, headingGain);
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_HEADING_TARGET, headingTracker.getTargetHeading());
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_HEADING_BIAS, headingTracker.getBias());
  EEPROM.update(EEPROM_ADDR::EE_ADDR_HEADING_CALIBRATED, headingTracker.isBiasCalibrated() ? 1 : 0);
}

void Penggerak::muatHeading() {
  if (EEPROM.read(EEPROM_ADDR::EE_ADDR_HEADING_MAGIC) == MAGIC_HEADING_VALID) {
    EEPROM.get(EEPROM_ADDR::EE_ADDR_HEADING_GAIN, headingGain);
    headingGain = constrain(headingGain, 0.0f, 10.0f);
    float target;
    EEPROM.get(EEPROM_ADDR::EE_ADDR_HEADING_TARGET, target);
    headingTracker.setTarget(target);
    float bias;
    EEPROM.get(EEPROM_ADDR::EE_ADDR_HEADING_BIAS, bias);
    headingTracker.setBias(bias);
    bool calibrated = EEPROM.read(EEPROM_ADDR::EE_ADDR_HEADING_CALIBRATED) != 0;
    // headingTracker internal state for calibrated - setBias already handles it
    (void)calibrated;
  }
}
#endif

// Private helper
float Penggerak::applyHeadingCorrection(float pidValue) {
#if ENABLE_HEADING_CORRECTION
  if (headingTracker.isEnabled()) {
    // updateHeading() called from loop() each iteration (including when stopped/calibrating).
    // Here we just apply the correction.
    float headingErr = headingTracker.getHeadingError();
    float headingCorrection = headingErr * headingGain;
    headingCorrection = constrain(headingCorrection, -HEADING_MAX_OFFSET, HEADING_MAX_OFFSET);
    pidValue += headingCorrection;
  }
#endif
  return pidValue;
}