// antarmuka.h — input manusia: tombol onboard + tepuk tangan + serial command.
#ifndef ANTARMUKA_H
#define ANTARMUKA_H

#include <Arduino.h>
#include "fwd.h"
#include "config.h"
#include "status.h"

class Antarmuka {
 public:
  Antarmuka(Status &st, SensorGaris &sen, Penggerak &g, Pengaman &am, LapTimer &lt, Mapper &mp);
  void cekTombol();   // non-blocking: tanpa delay, pakai cooldown
  void cekTepuk();    // 1x tepuk = toggle jalan/berhenti
  void cekSerial();   // parsing command serial: Kp/Ki/Kd/speed/ambang/calibrate/save/help

private:
  void mulai(const char *via);
  void berhenti(const char *via);
  void resetSemua();
  void parseCommand(char *cmd);
  void tuningSequence();
  void tampilkanStatusJalannya();

  // parseCommand helpers
  void parsePIDCommand(const char *key, const char *valStr, float val, bool hasVal);
  void parseSpeedCommand(const char *key, const char *valStr, float val, bool hasVal);
  void parseThresholdCommand(const char *key, const char *valStr, float val, bool hasVal);
  void parseCalibrateCommand(const char *valStr);
  void parseTuneCommand();
  void parseLapCommand(const char *valStr);
  void parseLapConfigCommand(const char *valStr);
  void parseSaveCommand();
  void parseLoadCommand();
  void parseModeCommand(const char *valStr);
  void parseRCCommand(const char *valStr);
  void parseFuzzyCommand(const char *valStr);
  void parseGainSchedulingCommand(const char *valStr);
  void parseHeadingCommand(const char *valStr);
  void parseHTargetCommand(const char *valStr);
  void parseAdaptiveCommand(const char *valStr);
  void parseMapClearCommand();
  void parsePathCommand(const char *valStr);
  void parseStartCommand();
  void parseStopCommand();
  void parseStatusCommand();
  void parseHelpCommand();

  Status &status;
  SensorGaris &sensor;
  Penggerak &gerak;
  Pengaman &aman;
  LapTimer &lapTimer;
  Mapper &mapper;

  unsigned long cooldownB1 = 0;
  unsigned long cooldownB2 = 0;
  // Atomic flag combo B1+B2 (volatile: diubah di cekTombol, dibaca di cekSerial/parseCommand)
  volatile bool comboActive = false;      // combo sedang berjalan
  unsigned long comboMulai = 0;            // ms saat combo dimulai
  bool comboSelesai = false;               // true = auto-tune sudah jalan
  unsigned long cekSuaraTerakhir = 0;
  unsigned long tepukTerakhir = 0;
  bool tepukSiap = true;
};

#endif
