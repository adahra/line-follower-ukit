// status.h — status jalan bersama, dioper antar modul via referensi.
#ifndef STATUS_H
#define STATUS_H

struct Status {
  bool jalan = false;
  bool modeSetelKd = false;
  unsigned long recoveryMulai = 0;  // 0 = tidak dalam mode recovery
};

#endif
