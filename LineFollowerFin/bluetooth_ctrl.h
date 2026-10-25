// bluetooth_ctrl.h — Bluetooth/BLE control untuk uKit Explore 2
// Mendukung: Serial Bluetooth (HM-10/ESP32) atau built-in uKit BT
#ifndef BLUETOOTH_CTRL_H
#define BLUETOOTH_CTRL_H

#include <Arduino.h>
#include "fwd.h"
#include "config.h"
#include "status.h"

// Tipe modul Bluetooth
enum BTModuleType {
  BT_NONE = 0,
  BT_HM10,        // HM-10/HC-08 BLE module via Serial
  BT_UKIT_BUILTIN // uKit built-in Bluetooth (jika tersedia)
};

// Struktur data telemetry untuk kirim via BT
struct BTTelemetry {
  unsigned long timestamp;
  float error;
  float pid;
  int speedDasar;
  int speedKiri;
  int speedKanan;
  int sensorKiri;
  int sensorKanan;
  float battery;
  int mode;
  bool jalan;
};

class BluetoothCtrl {
 public:
  BluetoothCtrl(Status &st, Penggerak &g, SensorGaris &sen, Pengaman &am, LapTimer &lt);

  // Inisialisasi
  void begin(unsigned long baud = 115200, BTModuleType type = BT_HM10);
  
  // Dipanggil setiap loop
  void update();

  // Kirim telemetry berkala
  void sendTelemetry(const BTTelemetry &tel);
  
  // Kirim status ringkas
  void sendStatus();
  
  // Kirim lap data
  void sendLapSummary();
  
  // Broadcast event
  void sendEvent(const char *event, const char *detail = "");

  // Set callback untuk command processing
  void setCommandCallback(void (*callback)(const char *cmd));

  // Cek apakah BT connected
  bool isConnected() const;

  // Enable/disable auto telemetry
  void setAutoTelemetry(bool enable, unsigned long intervalMs = 200);

 private:
  Status &status;
  Penggerak &gerak;
  SensorGaris &sensor;
  Pengaman &aman;
  LapTimer &lapTimer;

  BTModuleType moduleType = BT_NONE;
  bool connected = false;
  bool autoTelemetry = false;
  unsigned long telemetryInterval = 200;
  unsigned long lastTelemetry = 0;
  unsigned long lastCommandCheck = 0;
  unsigned long commandCheckInterval = 50;

  // Command buffer
  static const int CMD_BUFFER_SIZE = 64;  // Perintah BT proyek tidak melebihi 63 karakter
  char cmdBuffer[CMD_BUFFER_SIZE];
  int cmdBufferPos = 0;

  // Callback
  void (*commandCallback)(const char *cmd) = nullptr;

  // Hardware serial untuk BT (Serial1 atau Serial2 tergantung board)
  // uKit Explore 2: Serial1 biasanya untuk BT
  HardwareSerial *btSerial = nullptr;

  // Helper
  void processCommand(const char *cmd);
  void handleBTCommand(const char *cmd);
  void parseAndExecute(const char *cmd);
  bool readBTLine(char *buffer, int maxLen);
  void sendResponse(const char *format, ...);
  void sendResponse(const __FlashStringHelper *format, ...);
  void sendOK(const char *msg = "");
  void sendError(const char *msg = "");
};

#endif