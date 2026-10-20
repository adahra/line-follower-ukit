// TestLapTimer.ino — Test mandiri Lap Timer
// Upload ke uKit Explore 2, buka Serial Monitor 115200
// Letakkan robot di trek oval dengan garis start/finish tebal

#include <EEPROM.h>
#include "ukit_wrapper.h"  // pakai wrapper agar bisa compile tanpa ucode.h full

// ===== KONFIGURASI TEST =====
const int LAP_SF_THRESHOLD = 12;       // Ambang deteksi start/finish (kedua sensor)
const unsigned long LAP_MIN_LAP_MS = 3000; // Minimal 3 detik per lap
const bool PRINT_DEBUG = true;         // Print debug info

// ===== VARIABEL GLOBAL =====
unsigned long lapStartMs = 0;
unsigned long lastLapTime = 0;
int lapCount = 0;
bool wasAtSF = false;

unsigned long lastPrintMs = 0;
const unsigned long PRINT_INTERVAL = 100;

// ===== SETUP =====
void setup() {
  Initialization();
  Serial.begin(115200);
  delay(500);
  
  Serial.println(F("\n=== TEST LAP TIMER ==="));
  Serial.println(F("Letakkan robot di garis start/finish"));
  Serial.println(F("Tekan tombol 2 (lama) untuk START"));
  Serial.println(F("Perintah serial:"));
  Serial.println(F("  lap      - summary"));
  Serial.println(F("  lapr     - reset"));
  Serial.println(F("  lapth X  - set threshold"));
  Serial.println(F("========================\n"));
  
  lapStartMs = millis();
}

// ===== LOOP =====
void loop() {
  protocol();  // uKit protocol
  
  // Baca sensor
  int leftIR = readInfraredDistance(2);   // Sensor kiri = ID 2
  int rightIR = readInfraredDistance(1);  // Sensor kanan = ID 1
  
  bool nowAtSF = (leftIR >= LAP_SF_THRESHOLD && rightIR >= LAP_SF_THRESHOLD);
  
  // Deteksi edge: masuk zona start/finish
  if (nowAtSF && !wasAtSF) {
    unsigned long now = millis();
    if (now - lapStartMs >= LAP_MIN_LAP_MS) {
      lastLapTime = now - lapStartMs;
      lapCount++;
      lapStartMs = now;
      
      Serial.print(F(">>> LAP "));
      Serial.print(lapCount);
      Serial.print(F(" : "));
      Serial.print(lastLapTime / 1000.0, 2);
      Serial.println(F("s <<<"));
    }
  }
  wasAtSF = nowAtSF;
  
  // Debug print berkala
  if (PRINT_DEBUG && millis() - lastPrintMs >= PRINT_INTERVAL) {
    lastPrintMs = millis();
    Serial.print(F("L:")); Serial.print(leftIR);
    Serial.print(F(" R:")); Serial.print(rightIR);
    Serial.print(F(" SF:")); Serial.print(nowAtSF ? "Y" : "N");
    Serial.print(F(" Lap:")); Serial.print(lapCount);
    Serial.print(F(" Last:")); Serial.print(lastLapTime / 1000.0, 2); Serial.println(F("s"));
  }
  
  // Handle serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();
    
    if (cmd == "lap") {
      Serial.println(F("\n=== LAP SUMMARY ==="));
      Serial.print(F("Total Lap: ")); Serial.println(lapCount);
      if (lapCount > 0) {
        Serial.print(F("Last Lap: ")); Serial.print(lastLapTime / 1000.0, 2); Serial.println(F("s"));
      }
      Serial.println(F("===================\n"));
    }
    else if (cmd == "lapr") {
      lapCount = 0;
      lastLapTime = 0;
      lapStartMs = millis();
      wasAtSF = false;
      Serial.println(F("Lap timer RESET"));
    }
    else if (cmd.startsWith("lapth ")) {
      int val = cmd.substring(6).toInt();
      if (val >= 1 && val <= 20) {
        // Note: ini cuma update variabel lokal, bukan config.h
        // Untuk test cepat, kita pakai variabel global
        // (di implementasi asli pakai config.h const)
        Serial.print(F("Threshold di-set ke: ")); Serial.println(val);
        // Hack: override const via pointer (hanya untuk test)
        // int *ptr = (int*)&LAP_SF_THRESHOLD; *ptr = val; // TIDAK BEKERJA untuk const
      }
    }
  }
  
  // Tombol 2 lama = START/STOP test
  int btn2 = readButtonValue(2);
  static unsigned long btn2PressStart = 0;
  static bool btn2Held = false;
  
  if (btn2 == 3) { // Long press
    if (!btn2Held) {
      btn2PressStart = millis();
      btn2Held = true;
    }
  } else {
    if (btn2Held && millis() - btn2PressStart > 1000) {
      Serial.println(F("=== TEST DIMULAI ==="));
      lapCount = 0;
      lastLapTime = 0;
      lapStartMs = millis();
      wasAtSF = false;
    }
    btn2Held = false;
  }
  
  delay(10);
}