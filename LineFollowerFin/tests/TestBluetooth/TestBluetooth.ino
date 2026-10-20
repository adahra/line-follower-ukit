// TestBluetooth.ino — Test mandiri Bluetooth HM-10/HC-08
// Upload ke uKit Explore 2
// Hubungkan HM-10 ke Serial1: VCC->3.3V, GND->GND, TX->RX1, RX->TX1 (via voltage divider!)
// Pair HP -> buka "Serial Bluetooth Terminal" app

#include "ukit_wrapper.h"

#define BT_BAUD 9600        // HM-10 default baud rate (ubah ke 115200 setelah test)
#define TEST_BAUD 115200    // Serial Monitor baud

// ===== VARIABEL =====
bool btConnected = false;
unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_INTERVAL = 500; // 500ms untuk test
bool autoTelemetry = false;

char btCmdBuffer[64];
int btCmdPos = 0;

// ===== SETUP =====
void setup() {
  Initialization();
  
  // Serial Monitor (USB)
  Serial.begin(TEST_BAUD);
  delay(500);
  
  // Serial1 untuk HM-10
  Serial1.begin(BT_BAUD);
  delay(100);
  
  Serial.println(F("\n=== TEST BLUETOOTH HM-10 ==="));
  Serial.print(F("Baud rate Serial1: ")); Serial.println(BT_BAUD);
  Serial.println(F("Hubungkan HP ke HM-10, buka Serial Bluetooth Terminal"));
  Serial.println(F("Kirim 'AT' dari HP -> balasan 'OK'"));
  Serial.println(F("Perintah test:"));
  Serial.println(F("  start / stop"));
  Serial.println(F("  telemetry on / off"));
  Serial.println(F("  status"));
  Serial.println(F("  bat"));
  Serial.println(F("==============================\n"));
  
  // Test AT command
  Serial1.println(F("AT"));
  delay(200);
  if (Serial1.available()) {
    String resp = Serial1.readStringUntil('\n');
    resp.trim();
    if (resp == "OK") {
      btConnected = true;
      Serial.println(F(">>> HM-10 CONNECTED (OK)"));
    } else {
      Serial.print(F("HM-10 response: ")); Serial.println(resp);
    }
  } else {
    Serial.println(F(">>> HM-10 NO RESPONSE (cek wiring/baud)"));
  }
}

// ===== LOOP =====
void loop() {
  protocol();
  
  // === 1. Terima dari Bluetooth (HP -> uKit) ===
  while (Serial1.available()) {
    char c = Serial1.read();
    
    if (c == '\n' || c == '\r') {
      if (btCmdPos > 0) {
        btCmdBuffer[btCmdPos] = '\0';
        processBTCommand(btCmdBuffer);
        btCmdPos = 0;
      }
    } else if (btCmdPos < 63) {
      btCmdBuffer[btCmdPos++] = c;
    }
  }
  
  // === 2. Terima dari Serial Monitor (USB -> uKit) ===
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    
    if (cmd == "at") {
      // Forward AT command ke HM-10
      Serial1.println(F("AT"));
      Serial.println(F(">> Sent: AT"));
    }
    else if (cmd.startsWith("at ")) {
      Serial1.println(cmd.substring(3));
      Serial.print(F(">> Sent: ")); Serial.println(cmd.substring(3));
    }
    else if (cmd == "bt on") {
      autoTelemetry = true;
      Serial.println(F("Auto telemetry ON"));
    }
    else if (cmd == "bt off") {
      autoTelemetry = false;
      Serial.println(F("Auto telemetry OFF"));
    }
    else if (cmd == "bat") {
      // Simulasi baca battery (butuh uKit library)
      Serial.println(F("Battery: (butuh aman.tegangan())"));
    }
    else if (cmd == "help") {
      printHelp();
    }
  }
  
  // === 3. Auto Telemetry (uKit -> HP) ===
  if (autoTelemetry && btConnected && millis() - lastTelemetryMs >= TELEMETRY_INTERVAL) {
    lastTelemetryMs = millis();
    
    // Baca sensor untuk demo
    int leftIR = readInfraredDistance(2);
    int rightIR = readInfraredDistance(1);
    
    Serial1.print(F("TEL,"));
    Serial1.print(millis());
    Serial1.print(F(","));
    Serial1.print(leftIR);
    Serial1.print(F(","));
    Serial1.print(rightIR);
    Serial1.print(F(","));
    Serial1.print(random(50, 150)); // dummy speed
    Serial1.println();
  }
  
  delay(10);
}

// ===== PROSES COMMAND DARI BLUETOOTH ===
void processBTCommand(const char *cmd) {
  Serial.print(F("BT RX: ")); Serial.println(cmd);
  
  String s = String(cmd);
  s.trim();
  s.toLowerCase();
  
  if (s == "start") {
    Serial1.println(F("OK: Robot START (simulasi)"));
    Serial.println(F(">>> START command received"));
  }
  else if (s == "stop") {
    Serial1.println(F("OK: Robot STOP (simulasi)"));
    Serial.println(F(">>> STOP command received"));
  }
  else if (s == "status") {
    Serial1.print(F("STATUS,CONNECTED=")); Serial1.print(btConnected ? 1 : 0);
    Serial1.print(F(",TELEMETRY=")); Serial1.println(autoTelemetry ? 1 : 0);
    Serial.println(F(">>> Status sent"));
  }
  else if (s == "telemetry on") {
    autoTelemetry = true;
    Serial1.println(F("OK: Telemetry ON"));
    Serial.println(F(">>> Telemetry enabled"));
  }
  else if (s == "telemetry off") {
    autoTelemetry = false;
    Serial1.println(F("OK: Telemetry OFF"));
    Serial.println(F(">>> Telemetry disabled"));
  }
  else if (s == "bat") {
    Serial1.println(F("BAT,7.2V"));
    Serial.println(F(">>> Battery sent"));
  }
  else if (s == "help" || s == "?") {
    Serial1.println(F("CMDS: start, stop, status, telemetry on/off, bat, help"));
  }
  else if (s.startsWith("kp") || s.startsWith("kd") || s.startsWith("speed")) {
    Serial1.print(F("OK: ")); Serial1.println(s);
    Serial.print(F(">>> Param: ")); Serial.println(s);
  }
  else if (s == "at") {
    // AT command dari HP, forward ke Serial1 sudah di-handle di loop
  }
  else {
    Serial1.print(F("ERR: Unknown: ")); Serial1.println(cmd);
    Serial.print(F(">>> Unknown: ")); Serial.println(cmd);
  }
}

void printHelp() {
  Serial.println(F("\n=== SERIAL MONITOR COMMANDS ==="));
  Serial.println(F("at           - Test AT command"));
  Serial.println(F("at <cmd>     - Kirim AT command custom"));
  Serial.println(F("bt on/off    - Enable/disable auto telemetry"));
  Serial.println(F("bat          - Request battery (dummy)"));
  Serial.println(F("help         - This help"));
  Serial.println(F("==============================\n"));
}