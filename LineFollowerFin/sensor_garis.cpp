
#include "sensor_garis.h"
#include "penggerak.h"

void SensorGaris::resetFilter() {
  filtKiri = -1;
  filtKanan = -1;
}

// Filter eksponensial: redam 1 bacaan liar tanpa menambah delay.
int SensorGaris::bacaHalus(int id) {
  float &filt = (id == 2) ? filtKiri : filtKanan;
  int mentah = readInfraredDistance(id);
  if (filt < 0) {
    filt = mentah;
  } else {
    filt += ALPHA_SENSOR * (mentah - filt);
  }

  return (int)(filt + 0.5);
}

int SensorGaris::bacaKiri() {
  return bacaHalus(2);
}

int SensorGaris::bacaKanan() {
  return bacaHalus(1);
}

void SensorGaris::kalibrasi(Penggerak &gerak) {
  Serial.println("--- MEMULAI KALIBRASI SENSOR ---");
  int maksKiri = 0, minKiri = 20;
  int maksKanan = 0, minKanan = 20;

  setRgbledColor(255, 255, 0);
  delay(500);
  gerak.putarKalibrasi();

  unsigned long waktuMulai = millis();
  while (millis() - waktuMulai < 3000) {
    int sensorKanan = readInfraredDistance(1);
    int sensorKiri = readInfraredDistance(2);
    if (sensorKanan > maksKanan && sensorKanan <= 20) {
      maksKanan = sensorKanan;
    }

    if (sensorKiri > maksKiri && sensorKiri <= 20) {
      maksKiri = sensorKiri;
    }

    if (sensorKanan < minKanan && sensorKanan > 0) {
      minKanan = sensorKanan;
    }

    if (sensorKiri < minKiri && sensorKiri > 0) {
      minKiri = sensorKiri;
    }

    delay(10);
  }

  gerak.berhenti();

  AMBANG_KANAN = constrain((maksKanan + minKanan) / 2, 5, 14);
  AMBANG_KIRI = constrain((maksKiri + minKiri) / 2, 5, 14);
  Serial.print("Ambang kiri: ");
  Serial.print(AMBANG_KIRI);
  Serial.print(" kanan: ");
  Serial.println(AMBANG_KANAN);
  simpan();

  for (int i = 0; i < 3; i++) {
    setRgbledColor(0, 255, 0);
    delay(200);
    setRgbledColor(0, 0, 0);
    delay(200);
  }
}

void SensorGaris::simpan() {
  eepromPutHemat(ADDR_KP, Kp);
  eepromPutHemat(ADDR_KD, Kd);
  eepromPutHemat(ADDR_SPEED, kecepatanDasarMaks);
  eepromPutHemat(ADDR_AMBANG, AMBANG_KIRI);
  eepromPutHemat(ADDR_AMBANG_KANAN, AMBANG_KANAN);
  EEPROM.update(ADDR_EEPROM_CHECK, 123);
}

void SensorGaris::muat() {
  if (EEPROM.read(ADDR_EEPROM_CHECK) == 123) {
    EEPROM.get(ADDR_KP, Kp);
    EEPROM.get(ADDR_KD, Kd);
    EEPROM.get(ADDR_SPEED, kecepatanDasarMaks);
    EEPROM.get(ADDR_AMBANG, AMBANG_KIRI);
    EEPROM.get(ADDR_AMBANG_KANAN, AMBANG_KANAN);
    // Kompatibel format lama (satu ambang): sampah EEPROM dijepit ke rentang valid.
    AMBANG_KIRI = constrain(AMBANG_KIRI, 5, 14);
    AMBANG_KANAN = constrain(AMBANG_KANAN, 5, 14);
  }
}

void SensorGaris::cetak() {
  Serial.println("--- NILAI TERSIMPAN ---");
  Serial.print("EEPROM valid: ");
  Serial.println(EEPROM.read(ADDR_EEPROM_CHECK) == 123 ? "YA" : "TIDAK (pakai default kode)");
  Serial.print("Kp: ");
  Serial.println(Kp);
  Serial.print("Ki: ");
  Serial.print(Ki);
  Serial.println(" (tidak disimpan)");
  Serial.print("Kd: ");
  Serial.println(Kd);
  Serial.print("Speed maks: ");
  Serial.println(kecepatanDasarMaks);
  Serial.print("Speed min: ");
  Serial.print(kecepatanDasarMin);
  Serial.println(" (tidak disimpan)");
  Serial.print("Ambang kiri: ");
  Serial.println(AMBANG_KIRI);
  Serial.print("Ambang kanan: ");
  Serial.println(AMBANG_KANAN);
  Serial.println("-----------------------");
}
