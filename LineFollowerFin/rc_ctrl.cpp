// rc_ctrl.cpp — Implementasi Remote Control (RC) receiver
// Membaca sinyal PWM/PPM dari RC receiver pada digital pin.
// Channel 1 = throttle, channel 2 = steering.
#include "rc_ctrl.h"
#include "penggerak.h"
#include "sensor_garis.h"
#include "pengaman.h"
#include "lap_timer.h"
#include "ukit_macros.h"
#include <stdarg.h>

RcCtrl::RcCtrl(Status &st, Penggerak &g, SensorGaris &sen, Pengaman &am, LapTimer &lt)
    : status(st), gerak(g), sensor(sen), aman(am), lapTimer(lt) {}

void RcCtrl::begin(unsigned long baud, int pin, RCSource src) {
  rcBaud = baud;
  rcPin = pin;
  rcSource = src;

  if (rcSource == RC_PWM) {
    pinMode(rcPin, INPUT);
    lastPulseTime = millis();
    signalLost = true;
    Serial.println(F("RC: PWM mode, pin "));
    Serial.print(rcPin);
    Serial.println(F(" siap"));
  } else if (rcSource == RC_SERIAL) {
    Serial1.begin(rcBaud);
    lastSerialCheck = millis();
    signalLost = true;
    Serial.println(F("RC: Serial mode, baud "));
    Serial.print(rcBaud);
    Serial.println(F(" siap"));
  } else {
    Serial.println(F("RC: Tipe sumber tidak valid"));
  }
}

void RcCtrl::update() {
  if (rcSource == RC_PWM) {
    readPwmChannel();
  } else if (rcSource == RC_SERIAL) {
    unsigned long now = millis();
    if (now - lastSerialCheck >= serialCheckInterval) {
      lastSerialCheck = now;
      while (Serial1.available()) {
        char c = Serial1.read();
        if (c == '\n' || c == '\r') {
          if (rcCmdBufferPos > 0) {
            rcCmdBuffer[rcCmdBufferPos] = '\0';
            processSerialLine(rcCmdBuffer);
            rcCmdBufferPos = 0;
          }
        } else if (rcCmdBufferPos < RC_CMD_BUFFER_SIZE - 1) {
          rcCmdBuffer[rcCmdBufferPos++] = c;
        }
      }
    }
  }

  // Cek timeout sinyal
  if (millis() - lastPulseTime > RC_PULSE_TIMEOUT_MS) {
    if (!signalLost) {
      signalLost = true;
      Serial.println(F("RC: Sinyal hilang"));
    }
  }

  if (isEnabled()) {
    status.modeOperasi = MODE_RC;
    mapToSpeeds();
  }
}

unsigned long RcCtrl::readPulseWidth_us(int pin) {
  // pulseIn dengan timeout 2ms (period RC 20ms = 50Hz, pulse max 2ms)
  return pulseIn(pin, HIGH, 2000);
}

void RcCtrl::readPwmChannel() {
  unsigned long pulseThrottle = readPulseWidth_us(rcPin);
  if (pulseThrottle > 0) {
    lastThrottleUs = pulseThrottle;
    lastPulseTime = millis();
    if (signalLost) {
      signalLost = false;
      Serial.println(F("RC: Sinyal kembali"));
    }
  }

  // Baca channel steering: pulseIn pada pin yang sama dengan phase berbeda
  // untuk PPM sederhana: pulse berikutnya adalah steering
  unsigned long pulseSteering = readPulseWidth_us(rcPin);
  if (pulseSteering > 0) {
    lastSteeringUs = pulseSteering;
    lastPulseTime = millis();
    if (signalLost) {
      signalLost = false;
      Serial.println(F("RC: Sinyal kembali"));
    }
  }
}

void RcCtrl::mapToSpeed(int pulseUs, int &outSpeedKiri, int &outSpeedKanan) const {
  // Map pulse 1000-2000us ke -100..100
  int pct = map(pulseUs, RC_MIN_US, RC_MAX_US, -100, 100);
  pct = constrain(pct, -100, 100);
  outSpeedKiri = pct;
  outSpeedKanan = pct;
}

void RcCtrl::mapToSpeeds() {
  // Throttle: channel 1
  throttlePct = map(lastThrottleUs, RC_MIN_US, RC_MAX_US, RC_SCALE_MIN, RC_SCALE_MAX);
  throttlePct = constrain(throttlePct, 0, 100);

  // Steering: channel 2
  int steerRaw = map(lastSteeringUs, RC_MIN_US, RC_MAX_US, -100, 100);
  steerRaw = constrain(steerRaw, -100, 100);
  if (abs(steerRaw) < RC_DEADBAND_US / 10) {
    steerRaw = 0;
  }
  steeringPct = steerRaw * RC_STEER_GAIN;
  steeringPct = constrain(steeringPct, -100, 100);
}

void RcCtrl::getSpeeds(int &outSpeedKiri, int &outSpeedKanan) const {
  int baseSpeed = map(throttlePct, 0, 100, 0, kecepatanDasarMaks);
  int steerOffset = map(steeringPct, -100, 100, -kecepatanDasarMaks / 2, kecepatanDasarMaks / 2);

  outSpeedKiri = constrain(baseSpeed + steerOffset, 0, kecepatanDasarMaks);
  outSpeedKanan = constrain(baseSpeed - steerOffset, 0, kecepatanDasarMaks);
}

bool RcCtrl::isEnabled() const {
  return status.modeOperasi == MODE_RC;
}

void RcCtrl::setEnabled(bool on) {
  if (on) {
    status.modeOperasi = MODE_RC;
  } else {
    status.modeOperasi = MODE_NORMAL;
  }
}

void RcCtrl::setMode(ModeOperasi mode) {
  status.modeOperasi = mode;
}

int RcCtrl::getThrottle() const {
  return throttlePct;
}

int RcCtrl::getSteering() const {
  return steeringPct;
}

bool RcCtrl::isSignalLost() const {
  return signalLost;
}

void RcCtrl::reset() {
  throttlePct = 0;
  steeringPct = 0;
  lastThrottleUs = RC_CENTER_US;
  lastSteeringUs = RC_CENTER_US;
  signalLost = true;
  lastPulseTime = millis();
}

void RcCtrl::processSerialLine(const char *line) {
  // Format sederhana: "T,<throttle>,<steering>"
  if (line[0] == 'T' && line[1] == ',') {
    int t = atoi(line + 2);
    int s = 0;
    char *comma = strchr((char*)(line + 2), ',');
    if (comma) {
      s = atoi(comma + 1);
    }
    throttlePct = constrain(t, 0, 100);
    steeringPct = constrain(s, -100, 100);
    lastPulseTime = millis();
    if (signalLost) {
      signalLost = false;
      Serial.println(F("RC: Sinyal kembali"));
    }
  }
}
