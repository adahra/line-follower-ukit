// bluetooth_ctrl.cpp — Implementasi Bluetooth/BLE Control
#include "bluetooth_ctrl.h"
#include "penggerak.h"
#include "sensor_garis.h"
#include "pengaman.h"
#include "lap_timer.h"
#include "ukit_macros.h"
#include <stdarg.h>

BluetoothCtrl::BluetoothCtrl(Status &st, Penggerak &g, SensorGaris &sen, Pengaman &am, LapTimer &lt)
    : status(st), gerak(g), sensor(sen), aman(am), lapTimer(lt) {}

void BluetoothCtrl::begin(unsigned long baud, BTModuleType type) {
  moduleType = type;
  
  #if defined(ARDUINO_UKIT_EXPLORE2)
    // uKit Explore 2: Serial1 untuk Bluetooth
    btSerial = &Serial1;
  #elif defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA)
    btSerial = &Serial1;
  #elif defined(ESP32)
    btSerial = &Serial1;  // ESP32: Serial1 untuk BT
  #else
    // Default: Serial1 jika ada, fallback ke Serial
    #if defined(UBRR1H)
      btSerial = &Serial1;
    #else
      btSerial = &Serial;
    #endif
  #endif

  if (btSerial) {
    btSerial->begin(baud);
    delay(100);
    
    // Untuk HM-10: test AT command
    if (moduleType == BT_HM10) {
      btSerial->println(F("AT"));
      delay(100);
      if (btSerial->available()) {
        String resp = btSerial->readStringUntil('\n');
        if (resp.indexOf("OK") >= 0) {
          connected = true;
          Serial.println(F("BT: HM-10 connected"));
        }
      }
    } else if (moduleType == BT_UKIT_BUILTIN) {
      // uKit built-in BT init jika ada
      connected = true;
      Serial.println(F("BT: uKit built-in ready"));
    }
    
    if (!connected) {
      Serial.println(F("BT: Module tidak terdeteksi, cek wiring"));
    }
  } else {
    Serial.println(F("BT: Hardware serial tidak tersedia"));
  }
}

void BluetoothCtrl::update() {
  if (!btSerial || !connected) return;

  unsigned long now = millis();

  // Cek command dari BT
  if (now - lastCommandCheck >= commandCheckInterval) {
    lastCommandCheck = now;
    
    while (btSerial->available()) {
      char c = btSerial->read();
      
      if (c == '\n' || c == '\r') {
        if (cmdBufferPos > 0) {
          cmdBuffer[cmdBufferPos] = '\0';
          processCommand(cmdBuffer);
          cmdBufferPos = 0;
        }
      } else if (cmdBufferPos < CMD_BUFFER_SIZE - 1) {
        cmdBuffer[cmdBufferPos++] = c;
      }
    }
  }

  // Auto telemetry
  if (autoTelemetry && now - lastTelemetry >= telemetryInterval) {
    lastTelemetry = now;
    
    BTTelemetry tel = {
      now,
      gerak.errorTerakhir(),  // Perlu akses ke error current
      0,  // PID value - perlu akses
      kecepatanDasar,  // global variable
      0, 0,  // speed kiri/kanan - perlu akses
      sensor.bacaKiri(),
      sensor.bacaKanan(),
      aman.tegangan(),
      (int)status.modeOperasi,
      status.jalan
    };
    
    // Update dengan nilai aktual dari gerak
    // Karena field private, kita kirim yang ada
    sendTelemetry(tel);
  }
}

void BluetoothCtrl::sendTelemetry(const BTTelemetry &tel) {
  if (!btSerial || !connected) return;
  
  btSerial->print(F("TEL,"));
  btSerial->print(tel.timestamp);
  btSerial->print(F(","));
  btSerial->print(tel.error, 2);
  btSerial->print(F(","));
  btSerial->print(tel.pid, 2);
  btSerial->print(F(","));
  btSerial->print(tel.speedDasar);
  btSerial->print(F(","));
  btSerial->print(tel.speedKiri);
  btSerial->print(F(","));
  btSerial->print(tel.speedKanan);
  btSerial->print(F(","));
  btSerial->print(tel.sensorKiri);
  btSerial->print(F(","));
  btSerial->print(tel.sensorKanan);
  btSerial->print(F(","));
  btSerial->print(tel.battery, 1);
  btSerial->print(F(","));
  btSerial->print(tel.mode);
  btSerial->print(F(","));
  btSerial->println(tel.jalan ? 1 : 0);
}

void BluetoothCtrl::sendStatus() {
  if (!btSerial || !connected) return;
  
  btSerial->print(F("STATUS,"));
  btSerial->print(status.jalan ? F("RUNNING") : F("STOPPED"));
  btSerial->print(F(","));
  btSerial->print(status.modeOperasi == MODE_NORMAL ? F("NORMAL") : 
                  status.modeOperasi == MODE_LEARNING ? F("LEARNING") : F("RACING"));
  btSerial->print(F(","));
  btSerial->print(F("Kp:")); btSerial->print(Kp);
  btSerial->print(F(",Kd:")); btSerial->print(Kd);
  btSerial->print(F(",Ki:")); btSerial->print(Ki);
  btSerial->print(F(",Speed:")); btSerial->print(kecepatanDasarMaks);
  btSerial->print(F(",Bat:")); btSerial->print(aman.tegangan(), 1);
  btSerial->print(F(",Laps:")); btSerial->print(lapTimer.getLapCount());
  btSerial->println();
}

void BluetoothCtrl::sendLapSummary() {
  if (!btSerial || !connected) return;
  
  btSerial->println(F("LAPS:"));
  for (int i = 0; i < lapTimer.getLapCount(); i++) {
    // Akses laps array - perlu public method
    btSerial->print(F("LAP,"));
    btSerial->print(i + 1);
    btSerial->print(F(","));
    btSerial->print(lapTimer.getLastLapTime() / 1000.0, 2); // Simplified
    btSerial->println();
  }
  btSerial->print(F("BEST,"));
  btSerial->print(lapTimer.getBestLapTime() / 1000.0, 2);
  btSerial->println();
}

void BluetoothCtrl::sendEvent(const char *event, const char *detail) {
  if (!btSerial || !connected) return;
  
  btSerial->print(F("EVT,"));
  btSerial->print(event);
  if (detail && strlen(detail) > 0) {
    btSerial->print(F(","));
    btSerial->print(detail);
  }
  btSerial->println();
}

void BluetoothCtrl::setCommandCallback(void (*callback)(const char *cmd)) {
  commandCallback = callback;
}

bool BluetoothCtrl::isConnected() const {
  return connected && btSerial != nullptr;
}

void BluetoothCtrl::setAutoTelemetry(bool enable, unsigned long intervalMs) {
  autoTelemetry = enable;
  telemetryInterval = intervalMs;
}

void BluetoothCtrl::processCommand(const char *cmd) {
  // Echo ke Serial untuk debugging
  Serial.print(F("BT RX: "));
  Serial.println(cmd);
  
  // Handle BT-specific commands
  handleBTCommand(cmd);
  
  // Forward ke callback jika ada (untuk parsing command standar)
  if (commandCallback) {
    commandCallback(cmd);
  }
}

void BluetoothCtrl::handleBTCommand(const char *cmd) {
  char cmdCopy[CMD_BUFFER_SIZE];
  strncpy(cmdCopy, cmd, CMD_BUFFER_SIZE - 1);
  cmdCopy[CMD_BUFFER_SIZE - 1] = '\0';
  
  char *space = strchr(cmdCopy, ' ');
  char *key = cmdCopy;
  char *valStr = "";
  if (space) {
    *space = '\0';
    valStr = space + 1;
  }
  
  // Convert to lowercase
  for (char *p = key; *p; ++p) *p = tolower(*p);
  
  if (strcmp(key, "bt") == 0) {
    if (strcmp(valStr, "status") == 0) {
      sendStatus();
    } else if (strcmp(valStr, "telemetry") == 0) {
      if (space) {
        int interval = atoi(space + 1);
        setAutoTelemetry(true, interval);
        sendOK("Telemetry enabled");
      } else {
        sendOK("Usage: bt telemetry <interval_ms>");
      }
    } else if (strcmp(valStr, "stop") == 0) {
      setAutoTelemetry(false);
      sendOK("Telemetry stopped");
    } else if (strcmp(valStr, "laps") == 0) {
      sendLapSummary();
    } else if (strcmp(valStr, "reset") == 0) {
      lapTimer.reset();
      sendOK("Lap timer reset");
    } else {
      sendError("Unknown bt command");
    }
  }
  else if (strcmp(key, "start") == 0) {
    if (!status.jalan) {
      status.jalan = true;
      sendEvent("START", "via BT");
      sendOK("Started");
    } else {
      sendError("Already running");
    }
  }
  else if (strcmp(key, "stop") == 0) {
    if (status.jalan) {
      status.jalan = false;
      sendEvent("STOP", "via BT");
      sendOK("Stopped");
    } else {
      sendError("Already stopped");
    }
  }
  else if (strcmp(key, "mode") == 0) {
    if (strcmp(valStr, "normal") == 0) {
      status.modeOperasi = MODE_NORMAL;
      sendOK("Mode: NORMAL");
    } else if (strcmp(valStr, "learn") == 0) {
      status.modeOperasi = MODE_LEARNING;
      sendOK("Mode: LEARNING");
    } else if (strcmp(valStr, "race") == 0) {
      status.modeOperasi = MODE_RACING;
      sendOK("Mode: RACING");
    } else {
      sendError("Usage: mode normal|learn|race");
    }
  }
  else if (strcmp(key, "kp") == 0) {
    if (valStr[0]) {
      Kp = atof(valStr);
      sendOK("Kp set");
    } else {
      sendError("Usage: kp <value>");
    }
  }
  else if (strcmp(key, "kd") == 0) {
    if (valStr[0]) {
      Kd = atof(valStr);
      sendOK("Kd set");
    } else {
      sendError("Usage: kd <value>");
    }
  }
  else if (strcmp(key, "ki") == 0) {
    if (valStr[0]) {
      Ki = atof(valStr);
      sendOK("Ki set");
    } else {
      sendError("Usage: ki <value>");
    }
  }
  else if (strcmp(key, "speed") == 0) {
    if (valStr[0]) {
      kecepatanDasarMaks = constrain(atoi(valStr), SPEED_TOMBOL_MIN, SPEED_TOMBOL_MAKS);
      sendOK("Speed set");
    } else {
      sendError("Usage: speed <value>");
    }
  }
  else if (strcmp(key, "save") == 0) {
    sensor.simpan();
    sendEvent("SAVE", "EEPROM");
    sendOK("Saved to EEPROM");
  }
  else if (strcmp(key, "load") == 0) {
    sensor.muat();
    sendOK("Loaded from EEPROM");
  }
  else if (strcmp(key, "pid") == 0) {
    btSerial->print(F("PID,Kp:")); btSerial->print(Kp);
    btSerial->print(F(",Kd:")); btSerial->print(Kd);
    btSerial->print(F(",Ki:")); btSerial->println(Ki);
  }
  else if (strcmp(key, "help") == 0 || strcmp(key, "?") == 0) {
    sendResponse(F("BT Commands:"));
    sendResponse(F("  start/stop          - Mulai/berhenti robot"));
    sendResponse(F("  mode normal|learn|race - Set mode operasi"));
    sendResponse(F("  kp/kd/ki <val>      - Set PID"));
    sendResponse(F("  speed <val>         - Set max speed"));
    sendResponse(F("  save/load           - EEPROM"));
    sendResponse(F("  bt status           - Status BT"));
    sendResponse(F("  bt telemetry <ms>   - Auto telemetry"));
    sendResponse(F("  bt laps             - Lap summary"));
    sendResponse(F("  bt reset            - Reset lap timer"));
    sendResponse(F("  pid                 - Show PID"));
    sendResponse(F("  help/?              - This help"));
  }
  else {
    // Forward ke parser command standar jika tidak dikenali
    if (commandCallback) {
      commandCallback(cmd);
    } else {
      sendError("Unknown command");
    }
  }
}

bool BluetoothCtrl::readBTLine(char *buffer, int maxLen) {
  if (!btSerial || !connected) return false;
  
  int pos = 0;
  unsigned long start = millis();
  while (millis() - start < 100) {  // Timeout 100ms
    if (btSerial->available()) {
      char c = btSerial->read();
      if (c == '\n' || c == '\r') {
        if (pos > 0) {
          buffer[pos] = '\0';
          return true;
        }
      } else if (pos < maxLen - 1) {
        buffer[pos++] = c;
      }
    }
  }
  return false;
}

void BluetoothCtrl::sendResponse(const char *format, ...) {
  if (!btSerial || !connected) return;
  
  va_list args;
  va_start(args, format);
  char buffer[128];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  
  btSerial->print(F("> "));
  btSerial->println(buffer);
}

void BluetoothCtrl::sendResponse(const __FlashStringHelper *format, ...) {
  if (!btSerial || !connected) return;
  
  // Copy Flash string to RAM buffer
  char formatBuf[128];
  strcpy_P(formatBuf, (const char*)format);
  
  va_list args;
  va_start(args, format);
  char buffer[128];
  vsnprintf(buffer, sizeof(buffer), formatBuf, args);
  va_end(args);
  
  btSerial->print(F("> "));
  btSerial->println(buffer);
}

void BluetoothCtrl::sendOK(const char *msg) {
  if (!btSerial || !connected) return;
  btSerial->print(F("OK"));
  if (msg && strlen(msg) > 0) {
    btSerial->print(F(": "));
    btSerial->print(msg);
  }
  btSerial->println();
}

void BluetoothCtrl::sendError(const char *msg) {
  if (!btSerial || !connected) return;
  btSerial->print(F("ERR"));
  if (msg && strlen(msg) > 0) {
    btSerial->print(F(": "));
    btSerial->print(msg);
  }
  btSerial->println();
}