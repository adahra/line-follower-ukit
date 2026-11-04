#include "antarmuka.h"
#include "sensor_garis.h"
#include "penggerak.h"
#include "pengaman.h"
#include "lap_timer.h"
#include "mapping.h"
#include "ukit_extern.h"
#include "ukit_macros.h"
#include <string.h>
#include <ctype.h>

#if ENABLE_RC
#include "rc_ctrl.h"
// rc adalah global yang dideklarasikan di LineFollowerFin.ino (di dalam #if ENABLE_RC).
extern RcCtrl rc;
#endif

#if ENABLE_PATH_PLANNER
#include "path_planner.h"
// pathPlanner adalah global yang dideklarasikan di LineFollowerFin.ino (di dalam #if ENABLE_PATH_PLANNER).
extern PathPlanner pathPlanner;
#endif

Antarmuka::Antarmuka(Status &st, SensorGaris &sen, Penggerak &g, Pengaman &am, LapTimer &lt, Mapper &mp)
    : status(st), sensor(sen), gerak(g), aman(am), lapTimer(lt), mapper(mp) {}

void Antarmuka::resetSemua() {
  gerak.reset();
  sensor.resetFilter();
  aman.resetDeteksi();
  status.recoveryMulai = 0;
}

void Antarmuka::mulai(const char *via) {
  sensor.simpan();
  resetSemua();  // start bersih: tanpa hutang integral/derivatif run lama
  // Jangan reset runStartTime di sini — recordEvent/checkPrediction auto-
  // baseline agar offset waypoint konsisten dengan elapsed (lihat mapping.cpp).
  status.jalan = true;
  // Hanya mode Learning/Racing yang membutuhkan baseline timer untuk prediksi.
  if (status.modeOperasi == MODE_LEARNING || status.modeOperasi == MODE_RACING) {
    mapper.mulaiRun();
  }
  setEyelightLook(1, 0, 7, 0, 0, 254);
  setEyelightLook(2, 0, 7, 0, 0, 254);
  Serial.print(via);
  Serial.println(": JALAN");
}

void Antarmuka::berhenti(const char *via) {
  status.jalan = false;
  resetSemua();
  setEyelightLook(1, 0, 5, 254, 0, 0);
  setEyelightLook(2, 0, 5, 254, 0, 0);
  Serial.print(via);
  Serial.println(F(": BERHENTI"));
}

void Antarmuka::cekTombol() {
  unsigned long now = millis();
  int buttonState1 = readButtonValue(1);
  int buttonState2 = readButtonValue(2);

  // --- COMBO B1+B2 DICEK DULU, sebelum handler tombol tunggal ---
  // Tanpa ini, branch tombol tunggal tetap menambah Kp / ganti mode tiap
  // 300 ms dan terus me-reset cooldownB1 sehingga alur "tekan lagi dalam
  // 2 detik untuk Auto-Tune" hampir mustahil tercapai.
  // Alur: tahan keduanya >= 2 detik = kalibrasi + auto-tune;
  // lepas sebelum 2 detik = kalibrasi sensor saja.
  // Fuzzy toggle lewat serial command 'fuzzy on/off' (lebih andal).
  if (!status.jalan && buttonState1 != 0 && buttonState2 != 0) {
    if (comboMulai == 0) {
      comboMulai = now;
      comboSelesai = false;
      Serial.println(F("Combo B1+B2: tahan 2 detik = Auto-Tune, lepas lebih cepat = kalibrasi saja."));
    } else if (!comboSelesai && now - comboMulai >= 2000) {
      comboSelesai = true;
      Serial.println(F(">>> Memulai Kalibrasi + Auto-Tuning PID <<<"));
      tuningSequence();
    }
    return;  // combo aktif: tombol tunggal tidak diproses
  }

  // Tombol dilepas sebelum 2 detik: kalibrasi sensor saja (sekali per combo).
  if (comboMulai != 0) {
    if (!comboSelesai && buttonState1 == 0 && buttonState2 == 0) {
      Serial.println(F(">>> Kalibrasi sensor..."));
      sensor.kalibrasi(gerak);
      setRgbledColor(255, 255, 0);  // kuning selesai kalibrasi
      Serial.println(F("Kalibrasi selesai. LED kuning."));
    }
    comboMulai = 0;
    comboSelesai = false;
    return;  // frame lepas: jangan langsung proses tombol tunggal
  }

  // --- Non-blocking tombol detector ---
  // Deteksi edge dan cooldown untuk tombol tunggal
  if (buttonState1 != 0 && now - cooldownB1 >= COOLDOWN_TOMBOL_MS) {
    cooldownB1 = now;
    if (buttonState1 == 1) {
      // Mode cycling hanya saat robot berhenti dan BUKAN mode setel Kd
      if (!status.jalan && !status.modeSetelKd) {
        // Pindah mode: Normal -> Learning -> Racing -> RC -> Normal
        if (status.modeOperasi == MODE_NORMAL) {
          status.modeOperasi = MODE_LEARNING;
          Serial.println(F("Mode: LEARNING (Rekam Lintasan)"));
          setRgbledColor(255, 255, 0); // Kuning = learning
        } else if (status.modeOperasi == MODE_LEARNING) {
          status.modeOperasi = MODE_RACING;
          Serial.println(F("Mode: RACING (Gunakan Peta)"));
          setRgbledColor(0, 255, 0); // Hijau = racing
        } else if (status.modeOperasi == MODE_RACING) {
#if ENABLE_RC
          status.modeOperasi = MODE_RC;
          Serial.println(F("Mode: RC (Remote Control)"));
          setRgbledColor(0, 255, 255); // Cyan = RC
#else
          // Jangan pernah masuk MODE_RC bila fitur RC tidak dikompilasi:
          // loop tidak akan menggerakkan motor dan robot tampak mati.
          status.modeOperasi = MODE_NORMAL;
          Serial.println(F("Mode: NORMAL (RC dimatikan di config.h)"));
          setRgbledColor(0, 0, 0); // Hitam = normal
#endif
        } else if (status.modeOperasi == MODE_RC) {
          status.modeOperasi = MODE_NORMAL;
          Serial.println(F("Mode: NORMAL"));
          setRgbledColor(0, 0, 0); // Hitam = normal
        }
      } else if (!status.modeSetelKd) {
        Kp += 1.0;
      } else {
        Kd += 0.5;
      }
      // Label & nilai: tampilkan mode hanya saat berhenti dan BUKAN mode setel Kd.
      bool showMode = (!status.modeSetelKd && !status.jalan);
      Serial.print(showMode ? F("Mode: ") : (status.modeSetelKd ? F("Kd: ") : F("Kp: ")));
      if (showMode) {
        Serial.println(status.modeOperasi == MODE_NORMAL ? F("NORMAL")
                       : status.modeOperasi == MODE_LEARNING ? F("LEARNING")
                       : status.modeOperasi == MODE_RACING ? F("RACING")
                       : F("RC"));
      } else {
        Serial.println(status.modeSetelKd ? Kd : Kp);
      }
    } else if (buttonState1 == 2) {
      if (!status.modeSetelKd) {
        Kp -= 1.0;
        if (Kp < 0) {
          Kp = 0;
        }
      } else {
        Kd -= 0.5;
        if (Kd < 0) {
          Kd = 0;
        }
      }
      Serial.print(!status.modeSetelKd ? F("Kp: ") : F("Kd: "));
      Serial.println(!status.modeSetelKd ? Kp : Kd);
    } else if (buttonState1 == 3) {
      if (status.jalan) {
        berhenti("Tombol1");
      } else {
        status.modeSetelKd = !status.modeSetelKd;
        Serial.print(F("Mode setel Kd: "));
        Serial.println(status.modeSetelKd ? F("YA") : F("TIDAK"));
      }
    }
  }

  if (buttonState2 != 0 && now - cooldownB2 >= COOLDOWN_TOMBOL_MS) {
    cooldownB2 = now;
    if (buttonState2 == 1) {
      kecepatanDasarMaks += 10;
      if (kecepatanDasarMaks > SPEED_TOMBOL_MAKS) {
        kecepatanDasarMaks = SPEED_TOMBOL_MAKS;
      }
      Serial.print(F("Target Speed Maks: "));
      Serial.println(kecepatanDasarMaks);
    } else if (buttonState2 == 2) {
      kecepatanDasarMaks -= 10;
      if (kecepatanDasarMaks < SPEED_TOMBOL_MIN) {
        kecepatanDasarMaks = SPEED_TOMBOL_MIN;
      }
      Serial.print(F("Target Speed Maks: "));
      Serial.println(kecepatanDasarMaks);
    } else if (buttonState2 == 3) {
      if (!status.jalan) {
        mulai("Tombol2");
      }
    }
  }

  // --- Kalibrasi + Auto-Tuning via combo sudah ditangani di awal cekTombol ---
}

void Antarmuka::cekTepuk() {
  unsigned long now = millis();
  if (now - cekSuaraTerakhir < CEK_SUARA_MS) {
    return;
  }

  cekSuaraTerakhir = now;

  int suara = readSoundValue(SOUND_SENSOR_ID);
  if (tepukSiap && suara >= TEPUK_AMBANG &&
      now - tepukTerakhir >= TEPUK_COOLDOWN_MS) {
    tepukSiap = false;
    tepukTerakhir = now;
    if (status.jalan) {
      berhenti("Tepuk");
    } else {
      mulai("Tepuk");
    }
  } else if (!tepukSiap && suara < TEPUK_LEPAS) {
    tepukSiap = true;
  }
}

void Antarmuka::cekSerial() {
  // Parser non-blocking: jangan pakai Serial.readBytesUntil() yang bisa
  // menunggu timeout ~1 detik saat baris parsial datang (robot jadi buta).
  static char buf[64];
  static int len = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        // trim trailing whitespace
        while (len > 0 && (buf[len - 1] == ' ' || buf[len - 1] == '\t')) {
          buf[--len] = '\0';
        }
        if (len > 0) {
          parseCommand(buf);
        }
        len = 0;
      }
    } else if (len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

void Antarmuka::parseCommand(char *cmd) {
  // to lowercase
  for (char *p = cmd; *p; ++p) {
    *p = tolower(*p);
  }
  char *space = strchr(cmd, ' ');
  char *key = cmd;
  const char *valStr = "";
  if (space) {
    *space = '\0';
    valStr = space + 1;
    while (*valStr == ' ') {
      valStr++;
    }
  }
  float val = atof(valStr);
  bool hasVal = (valStr[0] != '\0');

  if (strcmp(key, "kp") == 0) {
    if (hasVal) Kp = val;
    Serial.print(F("Kp = "));
    Serial.println(Kp);
  } else if (strcmp(key, "ki") == 0) {
    if (hasVal) Ki = val;
    Serial.print(F("Ki = "));
    Serial.println(Ki);
  } else if (strcmp(key, "kd") == 0) {
    if (hasVal) Kd = val;
    Serial.print(F("Kd = "));
    Serial.println(Kd);
  } else if (strcmp(key, "speed") == 0) {
    if (hasVal) kecepatanDasarMaks = constrain((int)val, SPEED_TOMBOL_MIN, SPEED_TOMBOL_MAKS);
    Serial.print(F("Speed maks = "));
    Serial.println(kecepatanDasarMaks);
  } else if (strcmp(key, "speedmin") == 0) {
    if (hasVal) kecepatanDasarMin = constrain((int)val, 10, SPEED_TOMBOL_MAKS);
    Serial.print(F("Speed min = "));
    Serial.println(kecepatanDasarMin);
  } else if (strcmp(key, "ambang") == 0) {
    if (hasVal) {
      int v = constrain((int)val, 1, 20);
      AMBANG_KIRI = v;
      AMBANG_KANAN = v;
      Serial.print(F("Ambang kiri/kanan = "));
      Serial.println(v);
    } else {
      Serial.print(F("Ambang kiri = "));
      Serial.print(AMBANG_KIRI);
      Serial.print(F(" kanan = "));
      Serial.println(AMBANG_KANAN);
    }
  } else if (strcmp(key, "ambangk") == 0) {
    if (hasVal) AMBANG_KIRI = constrain((int)val, 1, 20);
    Serial.print(F("Ambang kiri = "));
    Serial.println(AMBANG_KIRI);
  } else if (strcmp(key, "ambangn") == 0) {
    if (hasVal) AMBANG_KANAN = constrain((int)val, 1, 20);
    Serial.print(F("Ambang kanan = "));
    Serial.println(AMBANG_KANAN);
  } else if (strcmp(key, "calibrate") == 0) {
    if (status.jalan) {
      Serial.println(F("ERROR: Robot jalan, hentikan dulu."));
    } else if (valStr[0] == '\0' || strcmp(valStr, "auto") == 0) {
      Serial.println(F("Kalibrasi AUTO dimulai via serial (robot berputar sendiri)..."));
      sensor.kalibrasi(gerak, false);
    } else if (strcmp(valStr, "manual") == 0) {
      Serial.println(F("Kalibrasi MANUAL dimulai: putar robot 360 derajat dengan tangan."));
      sensor.kalibrasi(gerak, true);
    } else {
      Serial.println(F("Penggunaan: calibrate [auto|manual]"));
      Serial.println(F("  auto   = robot berputar sendiri (default)"));
      Serial.println(F("  manual = motor diam, robot diputar dengan tangan"));
    }
  } else if (strcmp(key, "tune") == 0) {
    if (!status.jalan) {
      Serial.println(F("Memulai Kalibrasi + Auto-Tuning PID..."));
      tuningSequence();
    } else {
      Serial.println(F("ERROR: Robot jalan, hentikan dulu."));
    }
  } else if (strcmp(key, "lap") == 0) {
    if (strcmp(valStr, "reset") == 0) {
      lapTimer.reset();
      Serial.println(F("Lap timer direset."));
    } else if (strcmp(valStr, "summary") == 0) {
      lapTimer.printSummary();
    } else if (strcmp(valStr, "csv") == 0) {
      lapTimer.printCSV();
    } else {
      // Default: tampilkan summary
      lapTimer.printSummary();
    }
  } else if (strcmp(key, "lapconfig") == 0) {
    if (hasVal) {
      // Format: lapconfig <threshold> <minLapMs>
      // Untuk simplicity, parse manual
      int threshold = atoi(valStr);
      const char* space = strchr(valStr, ' ');
      if (space) {
        int minLap = atoi(space + 1);
        lapTimer.setThresholds(threshold, minLap);
      } else {
        lapTimer.setThresholds(threshold, LAP_MIN_LAP_MS);
      }
    } else {
      Serial.print(F("LapConfig: threshold=")); Serial.print(LAP_SF_THRESHOLD);
      Serial.print(F(" minLap=")); Serial.println(LAP_MIN_LAP_MS);
    }
  } else if (strcmp(key, "save") == 0) {
    sensor.simpan();
    mapper.simpan();  // peta waypoint learning ikut persisten
    mapper.debugPrint();
    Serial.println(F("Disimpan ke EEPROM."));
  } else if (strcmp(key, "load") == 0) {
    sensor.muat();
    mapper.muat();
    mapper.debugPrint();
    sensor.cetak();
    Serial.print(F("Waypoint tersimpan: "));
    Serial.println(mapper.getWaypointCount());
  } else if (strcmp(key, "mode") == 0) {
    if (status.jalan) {
      Serial.println(F("ERROR: Tidak dapat mengubah mode saat robot sedang jalan. Berhentikan dulu."));
    } else if (strcmp(valStr, "learn") == 0) {
      status.modeOperasi = MODE_LEARNING;
      Serial.println(F("Mode: LEARNING (Rekam Lintasan)"));
    } else if (strcmp(valStr, "race") == 0) {
      status.modeOperasi = MODE_RACING;
      Serial.println(F("Mode: RACING (Gunakan Peta)"));
    } else if (strcmp(valStr, "rc") == 0) {
#if ENABLE_RC
      status.modeOperasi = MODE_RC;
      Serial.println(F("Mode: RC (Remote Control)"));
#else
      Serial.println(F("ERROR: RC dimatikan (ENABLE_RC=false di config.h)."));
#endif
    } else {
      status.modeOperasi = MODE_NORMAL;
      Serial.println(F("Mode: NORMAL"));
    }
  } else if (strcmp(key, "rc") == 0) {
#if ENABLE_RC
    if (strcmp(valStr, "on") == 0) {
      status.modeOperasi = MODE_RC;
      Serial.println(F("RC mode: ON (MODE_RC)"));
    } else if (strcmp(valStr, "off") == 0) {
      status.modeOperasi = MODE_NORMAL;
      Serial.println(F("RC mode: OFF (kembali ke MODE_NORMAL)"));
    } else if (strcmp(valStr, "status") == 0) {
      Serial.print(F("RC Throttle: ")); Serial.print(rc.getThrottle());
      Serial.print(F("% | Steering: ")); Serial.print(rc.getSteering());
      Serial.print(F("% | Signal: "));
      Serial.println(rc.isSignalLost() ? F("HILANG") : F("OK"));
    } else {
      Serial.println(F("Penggunaan: rc on | rc off | rc status"));
    }
#else
    Serial.println(F("RC tidak aktif (ENABLE_RC=false)."));
#endif
  } else if (strcmp(key, "fuzzy") == 0) {
#if ENABLE_FUZZY_PID
    if (strcmp(valStr, "on") == 0) {
      gerak.setFuzzyEnabled(true);
      Serial.println(F("Fuzzy PID: ON (adaptive gains aktif)"));
    } else if (strcmp(valStr, "off") == 0) {
      gerak.setFuzzyEnabled(false);
      Serial.println(F("Fuzzy PID: OFF (static gains dari config/EEPROM)"));
    } else {
      // Tampilkan status saja
      Serial.print(F("Fuzzy PID: "));
      Serial.println(gerak.isFuzzyEnabled() ? F("ON") : F("OFF"));
      Serial.println(F("Penggunaan: fuzzy on | fuzzy off | fuzzy"));
    }
#else
    Serial.println(F("Fuzzy PID tidak aktif (ENABLE_FUZZY_PID=false)."));
#endif
  } else if (strcmp(key, "gs") == 0) {
    // Gain Scheduling
#if ENABLE_GAIN_SCHEDULING
    if (strcmp(valStr, "on") == 0) {
      gerak.setGainScheduling(true);
      Serial.println(F("Gain Scheduling: ON"));
    } else if (strcmp(valStr, "off") == 0) {
      gerak.setGainScheduling(false);
      Serial.println(F("Gain Scheduling: OFF"));
    } else {
      Serial.print(F("Gain Scheduling: "));
      Serial.println(gerak.isGainSchedulingEnabled() ? F("ON") : F("OFF"));
      Serial.print(F("Kp sched: ")); Serial.print(gerak.getGainScheduleKp(), 2);
      Serial.print(F(" | Kd sched: ")); Serial.print(gerak.getGainScheduleKd(), 2);
      Serial.print(F(" | Ki sched: ")); Serial.println(gerak.getGainScheduleKi(), 2);
      Serial.println(F("Penggunaan: gs on | gs off | gs"));
    }
#else
    Serial.println(F("Gain Scheduling tidak aktif (ENABLE_GAIN_SCHEDULING=false)."));
#endif
  } else if (strcmp(key, "heading") == 0) {
    // IMU Heading Correction
#if ENABLE_HEADING_CORRECTION
    if (strcmp(valStr, "on") == 0) {
      gerak.setHeadingCorrection(true);
      Serial.println(F("Heading correction: ON"));
    } else if (strcmp(valStr, "off") == 0) {
      gerak.setHeadingCorrection(false);
      Serial.println(F("Heading correction: OFF"));
    } else if (strcmp(valStr, "reset") == 0) {
      gerak.resetHeading();
      Serial.println(F("Heading direset ke 0 (arah sekarang = lurus)."));
    } else if (strcmp(valStr, "cal") == 0) {
      if (!status.jalan) {
        Serial.println(F("Kalibrasi bias gyro... diamkan robot 1.5 detik."));
        gerak.calibrateHeadingBias();
      } else {
        Serial.println(F("ERROR: Hentikan robot dulu untuk kalibrasi heading."));
      }
    } else if (strcmp(valStr, "save") == 0) {
      gerak.simpanHeading();
      Serial.println(F("Heading correction disimpan ke EEPROM."));
    } else {
      Serial.print(F("Heading: ")); Serial.print(gerak.getHeading(), 1);
      Serial.print(F("° | Target: ")); Serial.print(gerak.getTargetHeading(), 1);
      Serial.print(F("° | Error: ")); Serial.print(gerak.getHeadingError(), 1);
      Serial.print(F("° | Correction: "));
      Serial.println(gerak.isHeadingCorrectionEnabled() ? F("ON") : F("OFF"));
      Serial.println(F("Penggunaan: heading on|off|reset|cal|save"));
    }
#else
    Serial.println(F("Heading correction tidak aktif (ENABLE_HEADING_CORRECTION=false)."));
#endif
  } else if (strcmp(key, "htarget") == 0) {
#if ENABLE_HEADING_CORRECTION
    if (hasVal) gerak.setHeadingTarget(val);
    Serial.print(F("Heading target = "));
    Serial.println(gerak.getTargetHeading());
#else
    Serial.println(F("Heading correction tidak aktif."));
#endif
  } else if (strcmp(key, "adapt") == 0) {
    // Adaptive Threshold
#if ENABLE_ADAPTIVE_THRESHOLD
    if (strcmp(valStr, "on") == 0) {
      sensor.setAdaptiveThreshold(true);
      Serial.println(F("Adaptive Threshold: ON"));
    } else if (strcmp(valStr, "off") == 0) {
      sensor.setAdaptiveThreshold(false);
      Serial.println(F("Adaptive Threshold: OFF"));
    } else if (strcmp(valStr, "reset") == 0) {
      sensor.resetAdaptive();
      Serial.println(F("Adaptive threshold direset ke base."));
    } else if (strcmp(valStr, "save") == 0) {
      sensor.simpanAdaptive();
      Serial.println(F("Adaptive threshold base disimpan ke EEPROM."));
    } else {
      Serial.print(F("Adaptive Threshold: "));
      Serial.println(sensor.isAdaptiveThresholdEnabled() ? F("ON") : F("OFF"));
      Serial.print(F("Ambang kiri dinamis: ")); Serial.print(sensor.ambangDinamis(2));
      Serial.print(F(" | kanan: ")); Serial.println(sensor.ambangDinamis(1));
      Serial.println(F("Penggunaan: adapt on|off|reset|save"));
    }
#else
    Serial.println(F("Adaptive Threshold tidak aktif (ENABLE_ADAPTIVE_THRESHOLD=false)."));
#endif
  } else if (strcmp(key, "mapclear") == 0) {
    if (status.jalan) {
      Serial.println(F("ERROR: Robot sedang jalan, hentikan dulu sebelum mapclear."));
    } else {
      mapper.reset();
      Serial.println(F("Peta waypoint direset (mapclear)."));
    }
  } else if (strcmp(key, "path") == 0) {
#if ENABLE_PATH_PLANNER
    pathPlanner.handleCommand(valStr);
#else
    Serial.println(F("Path planner dinonaktifkan (ENABLE_PATH_PLANNER=false)."));
#endif
  } else if (strcmp(key, "start") == 0) {
    if (!status.jalan) {
      mulai("Serial");
    } else {
      Serial.println(F("Sudah jalan."));
    }
  } else if (strcmp(key, "stop") == 0) {
    if (status.jalan) {
      berhenti("Serial");
    } else {
      Serial.println(F("Sudah berhenti."));
    }
  } else if (strcmp(key, "status") == 0) {
    tampilkanStatusJalannya();
    Serial.println(F("ki <val>        : set Ki (contoh: ki 0.02)"));
    Serial.println(F("kd <val>        : set Kd (contoh: kd 1.8)"));
    Serial.println(F("speed <val>     : set speed maks 60-150"));
    Serial.println(F("speedmin <val>  : set speed min"));
    Serial.println(F("ambang <val>    : set ambang kiri & kanan (1-20)"));
    Serial.println(F("ambangk <val>   : set ambang kiri saja"));
    Serial.println(F("ambangn <val>   : set ambang kanan saja"));
    Serial.println(F("mode <val>      : set mode (learn/race/rc/normal)"));
    Serial.println(F("rc on|off|status: aktifkan/nonaktifkan/cek status RC"));
    Serial.println(F("fuzzy on|off   : aktifkan/nonaktifkan/cek Fuzzy adaptive PID"));
    Serial.println(F("gs on|off     : aktifkan/nonaktifkan/cek Gain Scheduling"));
    Serial.println(F("heading on|off|reset|cal|save : aktifkan/cek/reset/kalibrasi/save heading correction"));
    Serial.println(F("htarget <val> : set target heading (derajat)"));
    Serial.println(F("adapt on|off|reset|save : aktifkan/cek/reset/save Adaptive Threshold"));
    Serial.println(F("calibrate [auto|manual] : kalibrasi sensor; auto = robot berputar, manual = diputar tangan"));
    Serial.println(F("tune            : Auto-Tune PID via relay feedback (Ziegler-Nichols)"));
    Serial.println(F("lap [summary|csv|reset] : lap timer summary/CSV/reset"));
    Serial.println(F("lapconfig <th> [minMs] : set threshold start/finish & min lap time"));
#if ENABLE_PATH_PLANNER
    Serial.println(F("path map        : tampilkan grid map"));
    Serial.println(F("path find       : cari path ke goal"));
    Serial.println(F("path goal <x> <y> : set goal position"));
    Serial.println(F("path next       : waypoint berikutnya"));
    Serial.println(F("path obstacle <x> <y> : tambah obstacle"));
    Serial.println(F("path reset      : reset path planner"));
#endif
    Serial.println(F("save            : simpan ke EEPROM (termasuk waypoint learning)"));
    Serial.println(F("load            : muat dari EEPROM & cetak waypoint"));
    Serial.println(F("start           : mulai jalan"));
    Serial.println(F("stop            : berhenti"));
    Serial.println(F("status          : tampilkan status sedang ini"));
    Serial.println(F("mapclear        : hapus semua waypoint peta (harus berhenti)"));
    Serial.println(F("help / ?        : tampilkan ini"));
    Serial.println(F("-----------------------"));
    Serial.println();
    Serial.println(F("=== MODE via TOMBOL (saat berhenti) ==="));
    Serial.println(F("B1 pendek: Pindah mode Normal <-> Learning <-> Racing <-> RC"));
    Serial.println(F("B1 lama: Toggle mode Setel Kp/Kd"));
    Serial.println(F("B1+B2: Kalibrasi sensor (pendek) / Auto-Tuning (lama)"));
    Serial.println(F("Fuzzy PID: gunakan serial 'fuzzy on/off' untuk toggle"));
  } else {
    Serial.print(F("Perintah tidak dikenal: "));
    Serial.println(cmd);
    Serial.println(F("Ketik 'help' untuk daftar perintah."));
  }
}

// Fungsi tampilkan status jalanannya ke serial
void Antarmuka::tampilkanStatusJalannya() {
  Serial.println(F("--- STATUS SEKARANG ---"));
  Serial.print(F("Jalan: ")); Serial.println(status.jalan ? F("YA") : F("TIDAK"));
  Serial.print(F("Mode operasi: "));
  switch (status.modeOperasi) {
    case 0: Serial.println(F("NORMAL")); break;
    case 1: Serial.println(F("LEARNING")); break;
    case 2: Serial.println(F("RACING")); break;
    case 3: Serial.println(F("RC")); break;
  }
  Serial.print(F("modeSetelKd: ")); Serial.println(status.modeSetelKd ? F("YA") : F("TIDAK"));
  Serial.print(F("recoveryMulai: ")); Serial.print(status.recoveryMulai); Serial.println(F(" ms"));
  Serial.print(F("Jumlah waypoint: ")); Serial.println(mapper.getWaypointCount());
  
  // Sensor status
  Serial.print(F("Sensor kiri: ")); Serial.print(sensor.bacaKiri());
  Serial.print(F(" (ambang: ")); Serial.print(AMBANG_KIRI); Serial.print(F(")"));
  Serial.print(F(" | Sensor kanan: ")); Serial.print(sensor.bacaKanan());
  Serial.print(F(" (ambang: ")); Serial.print(AMBANG_KANAN); Serial.println(F(")"));
  
  // PID & Speed
  Serial.print(F("Kp: ")); Serial.print(Kp);
  Serial.print(F(" | Kd: ")); Serial.print(Kd);
  Serial.print(F(" | Ki: ")); Serial.print(Ki);
  Serial.print(F(" | SpeedMaks: ")); Serial.print(kecepatanDasarMaks);
  Serial.print(F(" | Dasar: ")); Serial.print(kecepatanDasar);
#if ENABLE_FUZZY_PID
  Serial.print(F(" | Fuzzy: ")); Serial.print(gerak.isFuzzyEnabled() ? F("ON") : F("OFF"));
#endif
  
  // LED status
  Serial.println(F(" --- AKHIR STATUS ---"));
}

// --- SEQUENCE: Kalibrasi + Auto-Tuning PID (Relay Feedback / Ziegler-Nichols) ---
// Alur: Kalibrasi sensor -> Relay feedback -> Hitung ZN -> Simpan ke EEPROM
void Antarmuka::tuningSequence() {
  Serial.println(F("--- MEMULAI AUTO-TUNE PID (Relay Feedback) ---"));

  // Step 1: Kalibrasi sensor
  setRgbledColor(255, 165, 0); // oranye
  Serial.println(F("Step 1: Kalibrasi sensor..."));
  sensor.kalibrasi(gerak);
  Serial.println(F("Kalibrasi selesai."));

  // Step 2: Relay feedback — amati osilasi
  setRgbledColor(255, 0, 255); // magenta
  Serial.println(F("Step 2: Relay feedback (amati osilasi)..."));

  // Simpan parameter lama
  float saveKp = Kp, saveKd = Kd, saveKi = Ki;
  Ki = 0.0;  // nonaktifkan I untuk relay test
  Kp = 0.0;
  Kd = 0.0;

  gerak.reset();
  status.jalan = true;
  unsigned long t0 = millis();
  bool lastSign = false;
  bool havePrevCrossing = false;
  unsigned long prevCrossingTime = 0;
  int cycleCount = 0;
  float maxAmplitude = 0;
  unsigned long firstCrossingTime = 0;

  while (millis() - t0 < AT_TIMEOUT_MS && cycleCount < AT_MAX_CYCLES) {
    if (aman.bateraiKritis() || aman.miring() || aman.halangan()) break;

    int bacaKiri = sensor.bacaKiri();
    int bacaKanan = sensor.bacaKanan();
    float error = (bacaKiri - bacaKanan) / 2.0;

    // Relay: output = +AMPLITUDE atau -AMPLITUDE berdasarkan tanda error
    float relayOut = (error > AT_RELAY_HYSTERESIS) ? -AT_RELAY_AMPLITUDE
                  : (error < -AT_RELAY_HYSTERESIS) ? AT_RELAY_AMPLITUDE : 0;
    gerak.jalankan((int)(kecepatanDasarMin - relayOut),
                   (int)(kecepatanDasarMin + relayOut));

    // Deteksi crossing (error berubah tanda)
    bool nowSign = (error > 0);
    if (cycleCount > 0 && nowSign != lastSign) {
      unsigned long now = millis();
      if (!havePrevCrossing) {
        prevCrossingTime = now;
        firstCrossingTime = now;
        havePrevCrossing = true;
      } else if (now - prevCrossingTime >= AT_MIN_PERIOD_MS) {
        cycleCount++;
        prevCrossingTime = now;
      }
    }
    lastSign = nowSign;
    if (fabs(error) > maxAmplitude) maxAmplitude = fabs(error);

    delay(10);
  }

  status.jalan = false;
  gerak.saatBerhenti();

  // Step 3: Hitung Ziegler-Nichols
  unsigned long Tu = 0;
  if (cycleCount >= 2 && havePrevCrossing) {
    Tu = (millis() - firstCrossingTime) / cycleCount;
  }
  float Ku = (maxAmplitude > 0) ? (4.0 * AT_RELAY_AMPLITUDE) / (3.14159 * maxAmplitude) : 0;

  Serial.print(F("Osilasi: ")); Serial.print(cycleCount);
  Serial.print(F(" siklus, Tu=")); Serial.print(Tu);
  Serial.print(F(" ms, Ku=")); Serial.println(Ku, 2);

  if (Ku > 0 && Tu > 0) {
    // Ziegler-Nichols PID: Kp = 0.6*Ku, Ti = Tu/2, Td = Tu/8.
    // PID di kode ini diskrit: I += error dan D = error-lastError, jadi
    // Ki_code = Kp*dt/Ti dan Kd_code = Kp*Td/dt (dt = PID_INTERVAL_MS).
    // Rumus per-detik langsung (implementasi lama) salah skala 100x:
    // Ki terlalu besar dan Kd terlalu kecil.
    float TuDetik = Tu / 1000.0;
    float dt = PID_INTERVAL_MS / 1000.0;
    Kp = constrain(0.6 * Ku, 0.1, 50.0);
    Ki = constrain((Kp * dt) / (TuDetik / 2.0), 0.0, 5.0);
    Kd = constrain((Kp * (TuDetik / 8.0)) / dt, 0.0, 20.0);
    Serial.print(F("ZN PID: Kp=")); Serial.print(Kp);
    Serial.print(F(", Ki=")); Serial.print(Ki);
    Serial.print(F(", Kd=")); Serial.println(Kd);
  } else {
    Kp = saveKp; Kd = saveKd; Ki = saveKi;
    Serial.println(F("Auto-tune gagal, pakai parameter lama."));
  }

  // Step 4: Simpan
  setRgbledColor(0, 255, 0); // hijau
  sensor.simpan();
  Serial.println(F("--- AUTO-TUNE SELESAI ---"));
  status.modeOperasi = MODE_NORMAL;
  status.jalan = false;
  setRgbledColor(0, 0, 0);
}
