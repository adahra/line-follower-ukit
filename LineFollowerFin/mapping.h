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
  unsigned long timeOffset; // Waktu sejak waypoint sebelumnya (ms)
  uint8_t type;
  uint8_t action;
};

class Mapper {
 public:
  Mapper();

  // Mode Learning: Rekam event yang terjadi
  void recordEvent(EventType type, ActionType action);

  // Mulai sesi learning: reset waypoint + catat waktu mulai
  void startLearning();

  // Mulai sesi racing: catat waktu mulai (waypoint tetap)
  void startRace();

  // Mode Racing: Prediksi event berikutnya berdasarkan waktu
  bool checkPrediction(unsigned long currentTime, EventType &outType, ActionType &outAction);

  void reset();
  int getWaypointCount() const;
  void debugPrint();

 private:
  static const int MAX_WAYPOINTS = 50;
  Waypoint waypoints[MAX_WAYPOINTS];
  int count = 0;
  unsigned long lastEventTime = 0;
  unsigned long raceStartMs = 0;  // waktu mulai sesi racing (referensi prediksi)
};

#endif
