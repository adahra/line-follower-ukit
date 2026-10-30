// mapping.h — Sistem memori lintasan untuk mode Racing.
#ifndef MAPPING_H
#define MAPPING_H

#include <Arduino.h>

enum EventType {
  EVENT_NONE = 0,
  EVENT_INTERSECTION, // Persimpangan/Garis melintang
  EVENT_FINISH
};

enum ActionType {
  ACT_NONE = 0,
  ACT_STRAIGHT,
  ACT_LEFT,
  ACT_RIGHT
};

struct Waypoint {
  EventType type;
  ActionType action;
  unsigned long timeOffset; // Waktu sejak waypoint sebelumnya (ms)
};

class Mapper {
 public:
  Mapper();
  
  // Mode Learning: Rekam event yang terjadi
  void recordEvent(EventType type, ActionType action);
  
  // Mode Racing: Prediksi event berikutnya berdasarkan waktu relatif sejak run.
  bool checkPrediction(unsigned long currentTime, EventType &outType, ActionType &outAction);

  void reset();
  int getWaypointCount() const;
  void debugPrint();
  // Baseline timer hitung sejak run dimulai (panggil saat racing start).
  // Jika tidak dipanggil, checkPrediction auto-baseline di panggilan pertama.
  void mulaiRun();

 private:
  static const int MAX_WAYPOINTS = 50;
  Waypoint waypoints[MAX_WAYPOINTS];
  int count = 0;
  unsigned long lastEventTime = 0;
  unsigned long runStartTime = 0;   // millis() saat run dimulai (baseline)
  int lastPredictedIndex = -1;      // tiap waypoint hanya diprediksi sekali
};

#endif
