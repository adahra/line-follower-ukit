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

void RcCtrl::begin(unsigned long baud, int pin, RCSource src, int steerPin) {
  rcBaud = baud;
  rcPin = pin;
  rcSteerPin = steerPin;
  rcSource = src;

  if (rcSource == RC_PWM) {
    pinMode(rcPin, INPUT);
    pinMode(rcSteerPin, INPUT);
    lastPulseTime = millis();
    signalLost = true;
    Serial.print(F("RC: PWM mode, throttle pin "));
    Serial.print(rcPin);
    Serial.print(F(", steering pin "));
    Serial.print(rcSteerPin);
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
    // Baca pulse HANYA saat RC benar-benar dipakai. pulseIn() bisa
    // memblokir sampai timeout, jadi jangan dipanggil saat robot
    // menjalankan mode line follower biasa.
    if (status.modeOperasi == MODE_RC && status.jalan) {
      readPwmChannel();
    }
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
  // Receiver 50 Hz: pulse muncul tiap 20 ms. Timeout 25 ms agar satu
  // periode penuh tertangkap, tapi tidak menunggu selamanya.
  return pulseIn(pin, HIGH, 25000);
}

void RcCtrl::readPwmChannel() {
  unsigned long pulseThrottle = readPulseWidth_us(rcPin);
  unsigned long pulseSteering = readPulseWidth_us(rcSteerPin);

  bool throttleValid = (pulseThrottle >= RC_MIN_US && pulseThrottle <= RC_MAX_US);
  bool steeringValid = (pulseSteering >= RC_MIN_US && pulseSteering <= RC_MAX_US);

  if (throttleValid) lastThrottleUs = pulseThrottle;
  if (steeringValid) lastSteeringUs = pulseSteering;

  // Sinyal dianggap hidup hanya jika KEDUA channel valid. Satu channel
  // hilang = failsafe, jangan memakai nilai lama tanpa batas.
  if (throttleValid && steeringValid) {
    lastPulseTime = millis();
    if (signalLost) {
      signalLost = false;
      Serial.println(F("RC: Sinyal kembali"));
    }
  }
}

void RcCtrl::mapToSpeeds() {
  // Throttle: -100 (pulse min) .. 0 (center) .. +100 (pulse max).
  int throttleUs = (int)lastThrottleUs;
  int throttleDelta = throttleUs - RC_CENTER_US;
  if (abs(throttleDelta) <= RC_DEADBAND_US) {
    throttlePct = 0;
  } else if (throttleDelta > 0) {
    throttlePct = map(throttleDelta, 0, RC_MAX_US - RC_CENTER_US, 0, 100);
  } else {
    throttlePct = map(throttleDelta, RC_MIN_US - RC_CENTER_US, 0, -100, 0);
  }
  throttlePct = constrain(throttlePct, -100, 100);

  // Steering: -100 (kiri) .. 0 (center) .. +100 (kanan).
  int steeringUs = (int)lastSteeringUs;
  int steeringDelta = steeringUs - RC_CENTER_US;
  int steerPct;
  if (abs(steeringDelta) <= RC_DEADBAND_US) {
    steerPct = 0;
  } else if (steeringDelta > 0) {
    steerPct = map(steeringDelta, 0, RC_MAX_US - RC_CENTER_US, 0, 100);
  } else {
    steerPct = map(steeringDelta, RC_MIN_US - RC_CENTER_US, 0, -100, 0);
  }
  steeringPct = constrain(steerPct * RC_STEER_GAIN, -100, 100);
}

void RcCtrl::getSpeeds(int &outSpeedKiri, int &outSpeedKanan) const {
  // Throttle menentukan kecepatan dasar (negatif = mundur), steering
  // menggeser kiri/kanan. Saat throttle 0, steering saja menghasilkan
  // pivot turn (satu roda maju, satu mundur).
  int baseSpeed = (throttlePct * kecepatanDasarMaks) / 100;
  int steerOffset = (steeringPct * (kecepatanDasarMaks / 2)) / 100;

  outSpeedKiri = constrain(baseSpeed + steerOffset, -kecepatanDasarMaks, kecepatanDasarMaks);
  outSpeedKanan = constrain(baseSpeed - steerOffset, -kecepatanDasarMaks, kecepatanDasarMaks);
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
