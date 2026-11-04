#include "sensor_garis.h"
#include "penggerak.h"
#include "ukit_extern.h"
#include "ukit_macros.h"

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
  Serial.println(F("--- MEMULAI KALIBRASI SENSOR ---"));
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

  AMBANG_KANAN = constrain((maksKanan + minKanan) / 2, 3, 18);
  AMBANG_KIRI = constrain((maksKiri + minKiri) / 2, 3, 18);
  Serial.print(F("Ambang kiri: "));
  Serial.print(AMBANG_KIRI);
  Serial.print(F(" kanan: "));
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
  eepromPutHemat(ADDR_KI, Ki);
  eepromPutHemat(ADDR_KD, Kd);
  eepromPutHemat(ADDR_SPEED, kecepatanDasarMaks);
  eepromPutHemat(ADDR_AMBANG, AMBANG_KIRI);
  eepromPutHemat(ADDR_AMBANG_KANAN, AMBANG_KANAN);
#if ENABLE_ADAPTIVE_THRESHOLD
  eepromPutHemat(ADDR_AMBANG_BASE, AMBANG_KIRI);
  eepromPutHemat(ADDR_AMBANG_KANAN_BASE, AMBANG_KANAN);
  EEPROM.update(ADDR_ADAPT_MAGIC, 66);
#endif
  EEPROM.update(ADDR_EEPROM_CHECK, 123);
}

void SensorGaris::muat() {
  if (EEPROM.read(ADDR_EEPROM_CHECK) == 123) {
    EEPROM.get(ADDR_KP, Kp);
    EEPROM.get(ADDR_KI, Ki);
    EEPROM.get(ADDR_KD, Kd);
    EEPROM.get(ADDR_SPEED, kecepatanDasarMaks);
    EEPROM.get(ADDR_AMBANG, AMBANG_KIRI);
    EEPROM.get(ADDR_AMBANG_KANAN, AMBANG_KANAN);
    // Kompatibel format lama (satu ambang): sampah EEPROM dijepit ke rentang valid.
    AMBANG_KIRI = constrain(AMBANG_KIRI, 3, 18);
    AMBANG_KANAN = constrain(AMBANG_KANAN, 3, 18);
#if ENABLE_ADAPTIVE_THRESHOLD
    // Base adaptive hanya dipakai bila magic-nya valid. Tanpa magic, EEPROM
    // kosong (0xFFFF) terbaca -1 dan menjepit ambang ke batas bawah 3.
    if (EEPROM.read(ADDR_ADAPT_MAGIC) == 66) {
      int baseKiri = 0, baseKanan = 0;
      EEPROM.get(ADDR_AMBANG_BASE, baseKiri);
      EEPROM.get(ADDR_AMBANG_KANAN_BASE, baseKanan);
      if (baseKiri >= 1 && baseKiri <= 20 && baseKanan >= 1 && baseKanan <= 20) {
        AMBANG_KIRI = constrain(baseKiri, 3, 18);
        AMBANG_KANAN = constrain(baseKanan, 3, 18);
      }
    }
    adaptAmbangKiri = AMBANG_KIRI;
    adaptAmbangKanan = AMBANG_KANAN;
#endif
  }
}

// --- Adaptive Threshold methods ---
#if ENABLE_ADAPTIVE_THRESHOLD
int SensorGaris::ambangDinamis(int id) const {
  return (id == 2) ? adaptAmbangKiri : adaptAmbangKanan;
}

void SensorGaris::setAdaptiveThreshold(bool on) {
  adaptiveThresholdEnabled = on;
  if (on) resetAdaptive();
}

bool SensorGaris::isAdaptiveThresholdEnabled() const {
  return adaptiveThresholdEnabled;
}

void SensorGaris::updateAmbangDinamis(int bacaKiri, int bacaKanan, int kecepatanDasar) {
  // Ambang dinamis = base * (1 + speedFactor), di-clamp.
  float speedFactor = 1.0 + (kecepatanDasar / 150.0) * ADAPT_SPEED_FACTOR;
  adaptAmbangKiri = constrain((int)(AMBANG_KIRI * speedFactor), ADAPT_THRESH_MIN, ADAPT_THRESH_MAX);
  adaptAmbangKanan = constrain((int)(AMBANG_KANAN * speedFactor), ADAPT_THRESH_MIN, ADAPT_THRESH_MAX);
  (void)bacaKiri;
  (void)bacaKanan;
}

void SensorGaris::resetAdaptive() {
  adaptAmbangKiri = AMBANG_KIRI;
  adaptAmbangKanan = AMBANG_KANAN;
}

void SensorGaris::simpanAdaptive() {
  eepromPutHemat(ADDR_AMBANG_BASE, AMBANG_KIRI);
  eepromPutHemat(ADDR_AMBANG_KANAN_BASE, AMBANG_KANAN);
  EEPROM.update(ADDR_ADAPT_MAGIC, 66);
}

void SensorGaris::muatAdaptive() {
  if (EEPROM.read(ADDR_ADAPT_MAGIC) != 66) {
    resetAdaptive();  // belum pernah disimpan: pakai ambang saat ini
    return;
  }

  int baseKiri = 0, baseKanan = 0;
  EEPROM.get(ADDR_AMBANG_BASE, baseKiri);
  EEPROM.get(ADDR_AMBANG_KANAN_BASE, baseKanan);
  if (baseKiri >= 1 && baseKiri <= 20 && baseKanan >= 1 && baseKanan <= 20) {
    AMBANG_KIRI = constrain(baseKiri, 3, 18);
    AMBANG_KANAN = constrain(baseKanan, 3, 18);
  }
  adaptAmbangKiri = AMBANG_KIRI;
  adaptAmbangKanan = AMBANG_KANAN;
}
#endif

void SensorGaris::cetak() {
  Serial.println(F("--- NILAI TERSIMPAN ---"));
  Serial.print(F("EEPROM valid: "));
  Serial.println(EEPROM.read(ADDR_EEPROM_CHECK) == 123 ? F("YA") : F("TIDAK (pakai default kode)"));
  Serial.print(F("Kp: "));
  Serial.println(Kp);
  Serial.print(F("Ki: "));
  Serial.println(Ki);
  Serial.print(F("Kd: "));
  Serial.println(Kd);
  Serial.print(F("Speed maks: "));
  Serial.println(kecepatanDasarMaks);
  Serial.print(F("Speed min: "));
  Serial.print(kecepatanDasarMin);
  Serial.println(F(" (tidak disimpan)"));
  Serial.print(F("Ambang kiri: "));
  Serial.println(AMBANG_KIRI);
  Serial.print(F("Ambang kanan: "));
  Serial.println(AMBANG_KANAN);
  Serial.println(F("-----------------------"));
}
