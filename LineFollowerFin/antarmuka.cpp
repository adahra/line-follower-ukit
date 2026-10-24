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
  status.jalan = true;

  // Inisialisasi mapper sesuai mode operasi
  if (status.modeOperasi == MODE_LEARNING) {
    mapper.startLearning();
  } else if (status.modeOperasi == MODE_RACING) {
    mapper.startRace();
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

  // --- B1+B2 bersamaan: Kalibrasi (tekan biasa) / Auto-Tuning (tekan lama) ---
  // Cek SEBELUM handler tombol tunggal agar tekan bersamaan tidak
  // "dimakan" oleh cooldown individual.
  if (buttonState1 != 0 && buttonState2 != 0 && !status.jalan) {
    cooldownB1 = now;
    cooldownB2 = now;
    if (buttonState1 == 3 || buttonState2 == 3) {
      // Tekan lama: Kalibrasi + Auto-Tuning PID
      Serial.println(F(">>> Memulai Kalibrasi + Auto-Tuning PID <<<"));
      tuningSequence();
    } else {
      // Tekan biasa: Kalibrasi sensor saja
      Serial.println(F(">>> Kalibrasi sensor <<<"));
      sensor.kalibrasi(gerak);
      setRgbledColor(255, 255, 0);
      Serial.println(F("Kalibrasi selesai."));
    }
    return;
  }

  // --- Non-blocking tombol detector ---
  // Deteksi edge dan cooldown untuk tombol tunggal
  if (buttonState1 != 0 && now - cooldownB1 >= COOLDOWN_TOMBOL_MS) {
    cooldownB1 = now;
    if (buttonState1 == 1) {
      // Mode cycling hanya saat robot berhenti dan BUKAN mode setel Kd
      if (!status.jalan && !status.modeSetelKd) {
        // Pindah mode: Normal -> Learning -> Racing -> Normal
        if (status.modeOperasi == MODE_NORMAL) {
          status.modeOperasi = MODE_LEARNING;
          Serial.println(F("Mode: LEARNING (Rekam Lintasan)"));
          setRgbledColor(255, 255, 0); // Kuning = learning
        } else if (status.modeOperasi == MODE_LEARNING) {
          status.modeOperasi = MODE_RACING;
          Serial.println(F("Mode: RACING (Gunakan Peta)"));
          setRgbledColor(0, 255, 0); // Hijau = racing
        } else if (status.modeOperasi == MODE_RACING) {
          status.modeOperasi = MODE_NORMAL;
          Serial.println(F("Mode: NORMAL"));
          setRgbledColor(0, 0, 0); // Hitam = normal
        }
      } else if (!status.modeSetelKd) {
        Kp += 1.0;
      } else {
        Kd += 0.5;
      }
      Serial.print(!status.modeSetelKd && !status.jalan ? F("Mode: ") : F("Kp: "));
      // Tampilkan mode saat ini
      if (!status.modeSetelKd && !status.jalan) {
        Serial.println(status.modeOperasi == MODE_NORMAL ? F("NORMAL") : status.modeOperasi == MODE_LEARNING ? F("LEARNING") : F("RACING"));
      } else {
        Serial.println(!status.modeSetelKd ? Kp : Kd);
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

  // --- Kalibrasi + Auto-Tuning: B1+B2 bersamaan ---
  // Deteksi awal tekan bersamaan
  if (buttonState1 != 0 && buttonState2 != 0 && !status.jalan &&
      now - cooldownB1 >= COOLDOWN_TOMBOL_MS &&
      now - cooldownB2 >= COOLDOWN_TOMBOL_MS) {
    // Catat waktu tekan pertama kali
    if (cooldownB1 == 0 && cooldownB2 == 0) {
      cooldownB1 = now;  // Tanda tombol pertama ditekan
      Serial.println(F("--- MEMULAI SEQUENCE: Press again for Auto-Tune ---"));
      Serial.println(F("Tekan lagi B1+B2 dalam 2 detik untuk Auto-Tuning"));
      Serial.println(F("Atau tunggu 2 detik untuk kalibrasi saja..."));
    } else {
      // Sudah pernah tekan, cek durini ini kali
      if (now - cooldownB1 < 2000) {
        // < 2 detik: Hanya kalibrasi sensor
        Serial.println(F(">>> Kalibrasi sensor BLU..."));
        cooldownB1 = 0;
        cooldownB2 = 0;
        sensor.kalibrasi(gerak);
        setRgbledColor(255, 255, 0); // Kuning selesai kalibrasi
        Serial.println(F("Kalibrasi selesai. LED kuning."));
      } else {
        // > 2 detik: Kalibrasi + Auto-Tuning PID
        Serial.println(F(">>> Memulai Kalibrasi + Auto-Tuning PID <<<"));
        cooldownB1 = 0;
        cooldownB2 = 0;
        // Mulai sequence: Kalibrasi -> Auto-Tuning -> Simpan
        tuningSequence();
      }
    }
    return;
  }

  if (buttonState1 != 0 && now - cooldownB1 >= COOLDOWN_TOMBOL_MS) {
    cooldownB1 = now;
    if (buttonState1 == 1) {
      if (!status.modeSetelKd) {
        Kp += 1.0;
      } else {
        Kd += 0.5;
      }

      Serial.print(!status.modeSetelKd ? F("Kp: ") : F("Kd: "));
      Serial.println(!status.modeSetelKd ? Kp : Kd);
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
  if (!Serial.available()) {
    return;
  }
  static char buf[64];
  int len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  if (len > 0) {
    buf[len] = '\0';
    // trim trailing whitespace
    while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == '\n' || buf[len - 1] == ' ')) {
      buf[--len] = '\0';
    }
    if (len > 0) {
      parseCommand(buf);
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
  char *valStr = "";
  if (space) {
    *space = '\0';
    valStr = space + 1;
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
    if (!status.jalan) {
      Serial.println(F("Kalibrasi dimulai via serial..."));
      sensor.kalibrasi(gerak);
    } else {
      Serial.println(F("ERROR: Robot jalan, hentikan dulu."));
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
      char* space = strchr(valStr, ' ');
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
    mapper.debugPrint();
    Serial.println(F("Disimpan ke EEPROM."));
  } else if (strcmp(key, "load") == 0) {
    sensor.muat();
    mapper.debugPrint();
    sensor.cetak();
    Serial.print(F("Waypoint tersimpan: "));
    Serial.println(mapper.getWaypointCount());
  } else if (strcmp(key, "mode") == 0) {
    if (strcmp(valStr, "learn") == 0) {
      status.modeOperasi = MODE_LEARNING;
      Serial.println(F("Mode: LEARNING (Rekam Lintasan)"));
    } else if (strcmp(valStr, "race") == 0) {
      status.modeOperasi = MODE_RACING;
      Serial.println(F("Mode: RACING (Gunakan Peta)"));
    } else {
      status.modeOperasi = MODE_NORMAL;
      Serial.println(F("Mode: NORMAL"));
    }
  } else if (strcmp(key, "mapclear") == 0) {
    // Note: mapper instance is in .ino, so we can't access it directly here 
    // unless we pass it to Antarmuka. 
    // For now, let's handle this via a special signal or just use the .ino logic.
    Serial.println(F("Gunakan command 'mapclear' (diimplementasikan di .ino)"));
  // } else if (strcmp(key, "path") == 0) {  // Optional: enable with PathPlanner
  //     pathPlanner.handleCommand(valStr);
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
    Serial.println(F("calibrate       : jalankan kalibrasi sensor (harus berhenti)"));
    Serial.println(F("tune            : kalibrasi + Auto-Tuning PID sekaligus"));
    Serial.println(F("lap [summary|csv|reset] : lap timer summary/CSV/reset"));
    Serial.println(F("lapconfig <th> [minMs] : set threshold start/finish & min lap time"));
    // Serial.println(F("path map        : tampilkan grid map"));      // Optional: PathPlanner
    // Serial.println(F("path find       : cari path ke goal"));       // Optional: PathPlanner
    // Serial.println(F("path goal <x> <y> : set goal position"));    // Optional: PathPlanner
    // Serial.println(F("path next       : waypoint berikutnya"));     // Optional: PathPlanner
    // Serial.println(F("path obstacle <x> <y> : tambah obstacle"));   // Optional: PathPlanner
    // Serial.println(F("path reset      : reset path planner"));      // Optional: PathPlanner
    Serial.println(F("save            : simpan ke EEPROM (termasuk waypoint learning)"));
    Serial.println(F("load            : muat dari EEPROM & cetak waypoint"));
    Serial.println(F("start           : mulai jalan"));
    Serial.println(F("stop            : berhenti"));
    Serial.println(F("status          : tampilkan status sedang ini"));
    Serial.println(F("help / ?        : tampilkan ini"));
    Serial.println(F("-----------------------"));
    Serial.println();
    Serial.println(F("=== MODE via TOMBOL (saat berhenti) ==="));
    Serial.println(F("B1 pendek: Pindah mode Normal <-> Learning <-> Racing"));
    Serial.println(F("B1 lama: Toggle mode Setel Kp/Kd"));
    Serial.println(F("B1+B2: Kalibrasi sensor (pendek) / Auto-Tuning (lama)"));
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
  
  // LED status
  Serial.println(F(" --- AKHIR STATUS ---"));
}

// --- SEQUENCE: Kalibrasi + Auto-Tuning PID ---
// Alur: Kalibrasi sensor -> Test respons PID -> Fine-tuning -> Simpan ke EEPROM
void Antarmuka::tuningSequence() {
  unsigned long sequenceStart = millis();
  const unsigned long SEQ_TIMEOUT = 30000; // 30 detik total timeout
  
  Serial.println(F("--- MEMULAI SEQUENCE KALIBRASI + AUTO-TUNING ---"));
  
  // Step 1: Kalibrasi sensor
  setRgbledColor(255, 165, 0); // oranye = tahap kalibrasi
  Serial.println(F("Step 1: Kalibrasi sensor..."));
  sensor.kalibrasi(gerak);
  Serial.println(F("Kalibrasi selesai."));
  
  // Step 2: Auto-Tuning PID sederhana (grid search)
  setRgbledColor(255, 0, 255); // magenta = tahap auto-tuning
  Serial.println(F("Step 2: Auto-Tuning PID (grid search)..."));
  
  // Parameter grid untuk dicoba
  float kpValues[] = {15.0, 20.0, 25.0, 30.0};
  float kdValues[] = {0.5, 1.0, 1.5, 2.0};
  
  float bestKp = Kp;
  float bestKd = Kd;
  float bestScore = 9999.0; // Score lebih rendah = lebih baik
  
  // Simpan parameter lama
  float originalKp = Kp;
  float originalKd = Kd;
  
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      // Cek timeout
      if (millis() - sequenceStart > SEQ_TIMEOUT) {
        Serial.println(F("Timeout sequence!"));
        break;
      }
      
      float testKp = kpValues[i];
      float testKd = kdValues[j];
      
      // Terapkan parameter uji
      Kp = testKp;
      Kd = testKd;
      Ki = 0.01; // Ki tetap kecil
      
      // Jalankan robot sebentar untuk test respons
      status.jalan = true;
      unsigned long testStart = millis();
      const unsigned long TEST_DURATION = 5000; // 5 detik test
      
      float totalError = 0;
      int errorSamples = 0;
      bool oscillating = false;
      float prevError = 0;
      
      while (millis() - testStart < TEST_DURATION) {
        if (!status.jalan) break;
        
        // Baca sensor dan hitung PID (dengan interval 10ms)
        int bacaKiri = sensor.bacaKiri();
        int bacaKanan = sensor.bacaKanan();
        float error = (bacaKiri - bacaKanan) / 2.0;
        
        // Hitung P dan D sederhana
        float P = error;
        float D = error - prevError;
        prevError = error;
        totalError += abs(error);
        errorSamples++;
        
        // Deteksi oscillasi: error berubah tanda berkali-kali
        if ( (error > 0 && prevError < 0) || (error < 0 && prevError > 0) ) {
          oscillating = true;
        }
        
        delay(10); // Approximate PID interval
      }
      
      status.jalan = false;
      
      // Hitung skor: error rata-rata + penalty oscillasi
      float avgError = errorSamples > 0 ? totalError / errorSamples : 9999;
      float score = avgError;
      if (oscillating) score += 5.0; // Penalty besar buat oscillasi
      
      Serial.print(F("  Test Kp=")); Serial.print(testKp);
      Serial.print(F(", Kd=")); Serial.print(testKd);
      Serial.print(F(": avgError=")); Serial.print(avgError, 1);
      Serial.print(F(", oscill=")); Serial.print(oscillating ? "YA" : "TIDAK");
      Serial.println(F(", skor=")); Serial.print(score, 1);
      
      if (score < bestScore) {
        bestScore = score;
        bestKp = testKp;
        bestKd = testKd;
      }
    }
  }
  
  // Step 3: Terapkan parameter terbaik
  Serial.println(F("Step 3: Parameter terbaik..."));
  Kp = bestKp;
  Kd = bestKd;
  Serial.print(F("Kp terbaik: ")); Serial.println(Kp);
  Serial.print(F("Kd terbaik: ")); Serial.println(Kd);
  
  // Step 4: Simpan ke EEPROM
  setRgbledColor(0, 255, 0); // hijau = selesai
  sensor.simpan(); // Simpan Kp, Kd, ambang, speed ke EEPROM
  
  Serial.println(F("--- SEQUENCE SELESAI ---"));
  Serial.print(F("Parameter disimpan: Kp=")); Serial.print(Kp);
  Serial.print(F(", Kd=")); Serial.print(Kd);
  Serial.println(F(", ambang & speed ikut kalibrasi"));
  Serial.println(F("LED hijau = siap digunakan"));
  
  // Kembali ke mode normal
  status.modeOperasi = MODE_NORMAL;
  status.jalan = false;
  setRgbledColor(0, 0, 0);
}
