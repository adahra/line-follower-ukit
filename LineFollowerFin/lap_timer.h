// lap_timer.h — Lap timing & statistik untuk line follower
#ifndef LAP_TIMER_H
#define LAP_TIMER_H

#include <Arduino.h>
#include "config.h"

#define MAX_LAPS 15
// Hemat RAM saat lap timer dimatikan: array record tidak perlu 15 slot.
// Saat ENABLE_LAP_TIMER=false, update() memang tidak dipanggil dari loop.
#define LAP_STORAGE (ENABLE_LAP_TIMER ? MAX_LAPS : 1)

struct LapRecord {
  unsigned long lapTime;    // ms
  unsigned long timestamp;  // millis() saat finish
  float avgError;           // rata-rata error selama lap
  float maxError;           // error maksimum
  int avgSpeed;             // kecepatan dasar rata-rata
};

class LapTimer {
 public:
  LapTimer();

  // Inisialisasi & reset
  void begin();
  void reset();

  // Dipanggil setiap loop saat robot jalan. Nilai sensor sudah dibaca
  // pemanggil agar lap timer tidak membaca hardware/filter dua kali.
  // Return true jika lap baru terdeteksi
  bool update(int bacaKiri, int bacaKanan, int kecepatanDasar, float error);

  // Akses data
  int getLapCount() const;
  unsigned long getLastLapTime() const;
  unsigned long getBestLapTime() const;
  unsigned long getTotalTime() const;
  float getAverageLapTime() const;

  // Export & display
  void printSummary();           // ke Serial
  void printCSV();               // format CSV ke Serial
  void exportCSV();              // alias printCSV

  // Konfigurasi deteksi start/finish
  void setThresholds(int startFinishThreshold, unsigned long minLapMs);

 private:
  // State deteksi garis start/finish (kedua sensor HIGH bersamaan)
  bool wasAtStartFinish = false;
  unsigned long lastLapStartMs = 0;
  unsigned long lapStartMs = 0;

  // Statistik lap saat ini
  unsigned long currentLapSamples = 0;
  float currentLapErrorSum = 0;
  float currentLapMaxError = 0;
  long currentLapSpeedSum = 0;

  // Records
  LapRecord laps[LAP_STORAGE];
  int lapCount = 0;

  // Config
  int sfThreshold = 12;         // ambang deteksi start/finish (kedua sensor >= ini)
  unsigned long minLapMs = 3000; // minimal 3 detik per lap (anti false trigger)

  // Helper
  bool isAtStartFinish(int bacaKiri, int bacaKanan) const;
  void finishLap(float avgError, int avgSpeed);
};

#endif