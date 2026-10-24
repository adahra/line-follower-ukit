#include "mapping.h"

Mapper::Mapper() {
  reset();
}

void Mapper::reset() {
  count = 0;
  lastEventTime = 0;
  raceStartMs = 0;
  for (int i = 0; i < MAX_WAYPOINTS; i++) {
    waypoints[i] = {EVENT_NONE, ACT_NONE, 0};
  }
}

void Mapper::startLearning() {
  reset();
  lastEventTime = millis();  // referensi untuk offset event pertama
}

void Mapper::startRace() {
  raceStartMs = millis();  // referensi untuk prediksi time-based
}

void Mapper::recordEvent(EventType type, ActionType action) {
  if (count >= MAX_WAYPOINTS) return;

  unsigned long now = millis();
  // Offset = selisih dari event sebelumnya (atau dari startLearning).
  // lastEventTime di-set oleh startLearning() sebelum event pertama,
  // sehingga offset event pertama = waktu dari start learning.
  unsigned long offset = now - lastEventTime;

  waypoints[count] = {type, action, offset};
  count++;
  lastEventTime = now;
}

bool Mapper::checkPrediction(unsigned long currentTime, EventType &outType, ActionType &outAction) {
  if (count == 0) return false;

  // Gunakan waktu relatif terhadap startRace, bukan millis() absolut.
  unsigned long elapsed = currentTime - raceStartMs;
  unsigned long accumulatedTime = 0;
  for (int i = 0; i < count; i++) {
    accumulatedTime += waypoints[i].timeOffset;

    // Jika waktu relatif mendekati waktu waypoint (toleransi 100ms)
    if (elapsed >= accumulatedTime && elapsed < accumulatedTime + 100) {
      outType = (EventType)waypoints[i].type;
      outAction = (ActionType)waypoints[i].action;
      return true;
    }
  }
  return false;
}

int Mapper::getWaypointCount() const {
  return count;
}

void Mapper::debugPrint() {
  Serial.println(F("--- MAP DATA ---"));
  for (int i = 0; i < count; i++) {
    Serial.print(i);
    Serial.print(F(" | Type: "));
    Serial.print(waypoints[i].type);
    Serial.print(F(" | Act: "));
    Serial.print(waypoints[i].action);
    Serial.print(F(" | Offset: "));
    Serial.println(waypoints[i].timeOffset);
  }
  Serial.println(F("----------------"));
}
