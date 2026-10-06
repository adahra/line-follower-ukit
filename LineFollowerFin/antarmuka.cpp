
#include "antarmuka.h"
#include "sensor_garis.h"
#include "penggerak.h"
#include "pengaman.h"

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
  Serial.println(": BERHENTI");
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
    Serial.println("Kalibrasi manual dimulai...");
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

      Serial.print(!status.modeSetelKd ? "Kp: " : "Kd: ");
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

      Serial.print(!status.modeSetelKd ? "Kp: " : "Kd: ");
      Serial.println(!status.modeSetelKd ? Kp : Kd);
    } else if (buttonState1 == 3) {
      if (status.jalan) {
        berhenti("Tombol1");
      } else {
        status.modeSetelKd = !status.modeSetelKd;
        Serial.print("Mode setel Kd: ");
        Serial.println(status.modeSetelKd ? "YA" : "TIDAK");
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

      Serial.print("Target Speed Maks: ");
      Serial.println(kecepatanDasarMaks);
    } else if (buttonState2 == 2) {
      kecepatanDasarMaks -= 10;
      if (kecepatanDasarMaks < SPEED_TOMBOL_MIN) {
        kecepatanDasarMaks = SPEED_TOMBOL_MIN;
      }

      Serial.print("Target Speed Maks: ");
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
    tepukSiap = false;  // kunci sampai suara reda, agar 1 tepuk = 1 toggle
    tepukTerakhir = now;
    if (status.jalan) {
      berhenti("Tepuk");
    } else {
      mulai("Tepuk");
    }
  } else if (!tepukSiap && suara < TEPUK_LEPAS) {
    tepukSiap = true;  // suara reda, siap deteksi tepukan berikutnya
  }
}
