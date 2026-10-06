
#include "pengaman.h"
#include "sensor_garis.h"
#include "penggerak.h"

Pengaman::Pengaman(Status &st, Penggerak &g, SensorGaris &sen)
    : status(st), gerak(g), sensor(sen) {}

void Pengaman::resetDeteksi() {
  irBekuSejak = 0;
  accelRef = -1;
}

void Pengaman::resetSemua() {
  gerak.reset();
  sensor.resetFilter();
  resetDeteksi();
  status.recoveryMulai = 0;
}

float Pengaman::tegangan() {
  return readBatteryVoltage();
}

// Sekali terkunci, hanya power cycle (restart) yang membuka. Non-blocking.
bool Pengaman::bateraiKritis() {
  unsigned long now = millis();
  if (!bateraiLemah && now - cekBateraiTerakhir >= CEK_BATERAI_MS) {
    cekBateraiTerakhir = now;
    float v = readBatteryVoltage();
    if (v <= TEGANGAN_MIN) {
      bateraiLemah = true;
      status.jalan = false;
      resetSemua();
      gerak.berhenti();
      Serial.print("BATERAI LEMAH: ");
      Serial.print(v);
      Serial.println(" V - semua fungsi dihentikan. Restart robot.");
    }
  }

  if (bateraiLemah) {
    gerak.berhenti();
    setRgbledColor(255, 0, 0);
    return true;
  }

  return false;
}

void Pengaman::bacaIMU() {
  unsigned long now = millis();
  if (now - imuTerakhir < CEK_IMU_MS) {
    return;
  }
  imuTerakhir = now;
  IMU::read();
  imuRoll = IMU::getRoll();
  imuPitch = IMU::getPitch();
  float ax = IMU::getRawAccelX();
  float ay = IMU::getRawAccelY();
  float az = IMU::getRawAccelZ();
  imuAccelMag = sqrt(ax * ax + ay * ay + az * az);
}

// Robot miring/terangkat: hentikan motor selama tak rata.
// Tidak dikunci: jalan lagi sendiri saat kembali rata. Non-blocking.
bool Pengaman::miring() {
  bacaIMU();
  if (fabs(imuRoll) > TILT_MAKS_DERAJAT || fabs(imuPitch) > TILT_MAKS_DERAJAT) {
    gerak.berhenti();
    setRgbledColor(255, 0, 0);
    return true;
  }

  return false;
}

// Halangan depan <= ambang: berhenti + LED merah. Non-blocking.
bool Pengaman::halangan() {
  unsigned long now = millis();
  if (now - cekHalanganTerakhir >= CEK_HALANGAN_MS) {
    cekHalanganTerakhir = now;
    int jarak = readUltrasonicDistance(ULTRASONIC_ID);
    // 0 = bacaan tak valid (di luar jangkauan) -> abaikan, seperti contoh docs.
    bool baru = (jarak > 0 && jarak <= JARAK_BERHENTI_CM);
    if (baru != halanganDepan) {
      halanganDepan = baru;
      Serial.print("Halangan depan: ");
      Serial.println(halanganDepan ? "BERHENTI" : "JALAN");
    }
  }

  if (halanganDepan) {
    gerak.berhenti();
    setRgbledColor(255, 0, 0);
    return true;
  }

  return false;
}

// Robot macet: IR beku + accel stabil saat diperintah jalan.
// Dinyatakan macet: berhenti total, operator restart via tombol/tepuk.
bool Pengaman::macet(int bacaKiri, int bacaKanan) {
  bacaIMU();
  unsigned long now = millis();
  bool irBeku = (abs(bacaKiri - irRefKiri) <= STUCK_TOLERANSI_IR) &&
                (abs(bacaKanan - irRefKanan) <= STUCK_TOLERANSI_IR);
  bool accelDiam =
      (accelRef < 0) ||
      (fabs(imuAccelMag - accelRef) / (accelRef + 0.001) < STUCK_TOLERANSI_ACCEL);
  if (irBeku && accelDiam) {
    if (irBekuSejak == 0) {
      irBekuSejak = now;
    } else if (now - irBekuSejak > STUCK_DIAM_MS) {
      status.jalan = false;
      resetSemua();
      gerak.berhenti();
      setRgbledColor(255, 0, 0);
      Serial.println("MACET: sensor & gerak beku - berhenti total.");
      return true;
    }
  } else {
    irBekuSejak = 0;
    irRefKiri = bacaKiri;
    irRefKanan = bacaKanan;
    accelRef = imuAccelMag;
  }

  return false;
}

// Lampu otomatis: nyala putih saat gelap, mati saat terang.
// Histeresis 2 ambang mencegah kedip di batas terang/gelap.
void Pengaman::lampuOtomatis() {
  unsigned long now = millis();
  if (now - cekCahayaTerakhir < CEK_CAHAYA_MS) {
    return;
  }

  cekCahayaTerakhir = now;

  int lux = readLightValue(LIGHT_SENSOR_ID);
  if (!lampuGelapNyala && lux < AMBANG_GELAP_NYALA) {
    lampuGelapNyala = true;
    setEyelightAllPetals(EYELIGHT_KIRI, 255, 255, 255);
    setEyelightAllPetals(EYELIGHT_KANAN, 255, 255, 255);
  } else if (lampuGelapNyala && lux > AMBANG_GELAP_MATI) {
    lampuGelapNyala = false;
    setEyelightOff(EYELIGHT_KIRI);
    setEyelightOff(EYELIGHT_KANAN);
  }
}
