// status.h — status jalan bersama, dioper antar modul via referensi.
#ifndef STATUS_H
#define STATUS_H
#include "mapping.h"

enum SubState {
  SUBSTATE_TRACKING = 0,
  SUBSTATE_INTER_CREEP = 1,
  SUBSTATE_TURNING = 2
};


enum ModeOperasi {
  MODE_NORMAL = 0,
  MODE_LEARNING,
  MODE_RACING,
  MODE_RC
};

struct Status {
  bool jalan = false;
  bool modeSetelKd = false;
  unsigned long recoveryMulai = 0;  // 0 = tidak dalam mode recovery
  ModeOperasi modeOperasi = MODE_NORMAL;
  SubState subState = SUBSTATE_TRACKING;
  unsigned long subStateMulai = 0;
  ActionType subStateAksi = ACT_NONE;
  int targetAmbangBelok = 0;
};
#endif
