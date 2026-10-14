#include "antarmuka.h"
#include "sensor_garis.h"
#include "penggerak.h"
#include "pengaman.h"
#include "ukit_extern.h"
#include "ukit_macros.h"
#include <string.h>
#include <ctype.h>

Antarmuka::Antarmuka(Status &st, SensorGaris &sen, Penggerak &g, Pengaman &am)
    : status(st), sensor(sen), gerak(g), aman(am) {}

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

  // Kalibrasi manual: tekan 2 tombol bersamaan saat berhenti.
  // Dicek dulu agar tak bentrok dengan aksi tombol tunggal di bawah.
  if (buttonState1 != 0 && buttonState2 != 0 && !status.jalan &&
      now - cooldownB1 >= COOLDOWN_TOMBOL_MS &&
      now - cooldownB2 >= COOLDOWN_TOMBOL_MS) {
    cooldownB1 = now;
    cooldownB2 = now;
    Serial.println(F("Kalibrasi manual dimulai..."));
    sensor.kalibrasi(gerak);
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
  } else if (strcmp(key, "save") == 0) {
    sensor.simpan();
    Serial.println(F("Disimpan ke EEPROM."));
  } else if (strcmp(key, "load") == 0) {
    sensor.muat();
    sensor.cetak();
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
  } else if (strcmp(key, "help") == 0 || strcmp(key, "?") == 0) {
    Serial.println(F("--- SERIAL COMMANDS ---"));
    Serial.println(F("kp <val>        : set Kp (contoh: kp 22.5)"));
    Serial.println(F("ki <val>        : set Ki (contoh: ki 0.02)"));
    Serial.println(F("kd <val>        : set Kd (contoh: kd 1.8)"));
    Serial.println(F("speed <val>     : set speed maks 60-150"));
    Serial.println(F("speedmin <val>  : set speed min"));
    Serial.println(F("ambang <val>    : set ambang kiri & kanan (1-20)"));
    Serial.println(F("ambangk <val>   : set ambang kiri saja"));
    Serial.println(F("ambangn <val>   : set ambang kanan saja"));
    Serial.println(F("calibrate       : jalankan kalibrasi sensor (harus berhenti)"));
    Serial.println(F("save            : simpan ke EEPROM"));
    Serial.println(F("load            : muat dari EEPROM & cetak"));
    Serial.println(F("start           : mulai jalan"));
    Serial.println(F("stop            : berhenti"));
    Serial.println(F("help / ?        : tampilkan ini"));
    Serial.println(F("-----------------------"));
  } else {
    Serial.print(F("Perintah tidak dikenal: "));
    Serial.println(cmd);
    Serial.println(F("Ketik 'help' untuk daftar perintah."));
  }
}
