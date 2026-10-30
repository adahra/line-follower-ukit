#include "mapping.h"

Mapper::Mapper() {
  reset();
}

void Mapper::reset() {
  count = 0;
  lastEventTime = 0;
  for (int i = 0; i < MAX_WAYPOINTS; i++) {
    waypoints[i] = {EVENT_NONE, ACT_NONE, 0};
  }
}

void Mapper::recordEvent(EventType type, ActionType action) {
  if (count >= MAX_WAYPOINTS) return;

  unsigned long now = millis();
  unsigned long offset = (count == 0) ? now : (now - lastEventTime);
  
  waypoints[count] = {type, action, offset};
  count++;
  lastEventTime = now;
}

bool Mapper::checkPrediction(unsigned long currentTime, EventType &outType, ActionType &outAction) {
  if (count == 0) return false;

  // Hitung total waktu sejak start untuk waypoint saat ini
  unsigned long accumulatedTime = 0;
  for (int i = 0; i < count; i++) {
    accumulatedTime += waypoints[i].timeOffset;
    
    // Jika waktu sekarang mendekati waktu waypoint (toleransi 100ms)
    if (currentTime >= accumulatedTime && currentTime < accumulatedTime + 100) {
      outType = waypoints[i].type;
      outAction = waypoints[i].action;
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
