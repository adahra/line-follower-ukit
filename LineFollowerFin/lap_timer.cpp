// lap_timer.cpp — Implementasi Lap Timer
#include "lap_timer.h"
#include "ukit_macros.h"

LapTimer::LapTimer() {
  reset();
}

void LapTimer::begin() {
  reset();
}

void LapTimer::reset() {
  wasAtStartFinish = false;
  lastLapStartMs = 0;
  lapStartMs = millis();

  currentLapSamples = 0;
  currentLapErrorSum = 0;
  currentLapMaxError = 0;
  currentLapSpeedSum = 0;

  lapCount = 0;
  for (int i = 0; i < MAX_LAPS; i++) {
    laps[i] = {0, 0, 0, 0, 0};
  }
}

bool LapTimer::isAtStartFinish(int bacaKiri, int bacaKanan) const {
  // Start/finish: kedua sensor mendeteksi garis kuat (persimpangan khusus)
  // Bisa dikonfigurasi: misal garis tebal di start/finish
  return (bacaKiri >= sfThreshold && bacaKanan >= sfThreshold);
}

bool LapTimer::update(SensorGaris &sensor, int kecepatanDasar, float error) {
  int bacaKiri = sensor.bacaKiri();
  int bacaKanan = sensor.bacaKanan();

  bool nowAtSF = isAtStartFinish(bacaKiri, bacaKanan);

  // Akumulasi statistik lap saat ini
  currentLapSamples++;
  currentLapErrorSum += fabs(error);
  if (fabs(error) > currentLapMaxError) currentLapMaxError = fabs(error);
  currentLapSpeedSum += kecepatanDasar;

  // Deteksi edge: baru masuk zona start/finish
  if (nowAtSF && !wasAtStartFinish) {
    unsigned long now = millis();

    // Cek minimal lap time (anti bounce / false trigger)
    if (now - lapStartMs >= minLapMs) {
      // Hitung rata-rata untuk lap ini
      float avgErr = currentLapSamples > 0 ? currentLapErrorSum / currentLapSamples : 0;
      int avgSpd = currentLapSamples > 0 ? currentLapSpeedSum / currentLapSamples : 0;

      finishLap(avgErr, avgSpd);

      // Reset untuk lap berikutnya
      lapStartMs = now;
      currentLapSamples = 0;
      currentLapErrorSum = 0;
      currentLapMaxError = 0;
      currentLapSpeedSum = 0;

      return true;  // Lap baru selesai
    }
  }

  wasAtStartFinish = nowAtSF;
  return false;
}

void LapTimer::finishLap(float avgError, int avgSpeed) {
  unsigned long now = millis();
  unsigned long lapTime = now - lapStartMs;

  if (lapCount < MAX_LAPS) {
    laps[lapCount] = {
      lapTime,
      now,
      avgError,
      currentLapMaxError,
      avgSpeed
    };
    lapCount++;

    // Print real-time ke Serial
    Serial.print(F("LAP "));
    Serial.print(lapCount);
    Serial.print(F(": "));
    Serial.print(lapTime / 1000.0, 2);
    Serial.print(F("s | Best: "));
    Serial.print(getBestLapTime() / 1000.0, 2);
    Serial.print(F("s | AvgErr: "));
    Serial.print(avgError, 1);
    Serial.print(F(" | Speed: "));
    Serial.println(avgSpeed);
  } else {
    Serial.println(F("LAP: Buffer penuh (MAX_LAPS tercapai)"));
  }
}

int LapTimer::getLapCount() const {
  return lapCount;
}

unsigned long LapTimer::getLastLapTime() const {
  if (lapCount == 0) return 0;
  return laps[lapCount - 1].lapTime;
}

unsigned long LapTimer::getBestLapTime() const {
  if (lapCount == 0) return 0;
  unsigned long best = laps[0].lapTime;
  for (int i = 1; i < lapCount; i++) {
    if (laps[i].lapTime < best) best = laps[i].lapTime;
  }
  return best;
}

unsigned long LapTimer::getTotalTime() const {
  if (lapCount == 0) return 0;
  return laps[lapCount - 1].timestamp - laps[0].timestamp;
}

float LapTimer::getAverageLapTime() const {
  if (lapCount == 0) return 0;
  unsigned long total = 0;
  for (int i = 0; i < lapCount; i++) total += laps[i].lapTime;
  return total / 1000.0 / lapCount;
}

void LapTimer::printSummary() {
  Serial.println(F("\n=== LAP TIMER SUMMARY ==="));
  Serial.print(F("Total Lap: ")); Serial.println(lapCount);
  if (lapCount > 0) {
    Serial.print(F("Best Lap: ")); Serial.print(getBestLapTime() / 1000.0, 2); Serial.println(F("s"));
    Serial.print(F("Last Lap: ")); Serial.print(getLastLapTime() / 1000.0, 2); Serial.println(F("s"));
    Serial.print(F("Avg Lap:  ")); Serial.print(getAverageLapTime(), 2); Serial.println(F("s"));
    Serial.print(F("Total:    ")); Serial.print(getTotalTime() / 1000.0, 2); Serial.println(F("s"));
    Serial.println(F("-------------------------"));
    for (int i = 0; i < lapCount; i++) {
      Serial.print(F("Lap ")); Serial.print(i + 1);
      Serial.print(F(": ")); Serial.print(laps[i].lapTime / 1000.0, 2); Serial.print(F("s"));
      Serial.print(F(" | Err:")); Serial.print(laps[i].avgError, 1);
      Serial.print(F(" | MaxErr:")); Serial.print(laps[i].maxError, 1);
      Serial.print(F(" | Spd:")); Serial.println(laps[i].avgSpeed);
    }
  }
  Serial.println(F("=========================\n"));
}

void LapTimer::printCSV() {
  Serial.println(F("lap,time_s,avg_error,max_error,avg_speed,timestamp_ms"));
  for (int i = 0; i < lapCount; i++) {
    Serial.print(i + 1);
    Serial.print(F(","));
    Serial.print(laps[i].lapTime / 1000.0, 3);
    Serial.print(F(","));
    Serial.print(laps[i].avgError, 2);
    Serial.print(F(","));
    Serial.print(laps[i].maxError, 2);
    Serial.print(F(","));
    Serial.print(laps[i].avgSpeed);
    Serial.print(F(","));
    Serial.println(laps[i].timestamp);
  }
}

void LapTimer::exportCSV() {
  printCSV();
}

void LapTimer::setThresholds(int startFinishThreshold, unsigned long minLapMs) {
  sfThreshold = constrain(startFinishThreshold, 5, 20);
  this->minLapMs = minLapMs;
  Serial.print(F("LapTimer: SF threshold=")); Serial.print(sfThreshold);
  Serial.print(F(" minLap=")); Serial.print(minLapMs); Serial.println(F("ms"));
}