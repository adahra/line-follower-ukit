// status.h — status jalan bersama, dioper antar modul via referensi.
#ifndef STATUS_H
#define STATUS_H

enum ModeOperasi {
  MODE_NORMAL = 0,
  MODE_LEARNING,
  MODE_RACING
};

struct Status {
  bool jalan = false;
  bool modeSetelKd = false;
  unsigned long recoveryMulai = 0;  // 0 = tidak dalam mode recovery
  ModeOperasi modeOperasi = MODE_NORMAL;

  // Riwayat loop sebelumnya: apakah robot mengikuti garis (satu sensor putih)
  // atau di luar garis (both putih = intersection/lost-line). Bedakan
  // intersection (dalamGaris=true) vs garis putus (dalamGaris=false).
  // DISET di setiap branch loop(); default true agar loop pertama setelah
  // start/jalan ulang tidak menganggap garis yang diinjak sebagai lost line.
  bool dalamGaris = true;
};

#endif
