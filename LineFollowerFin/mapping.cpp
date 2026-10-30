#include "mapping.h"

Mapper::Mapper() {
  reset();
}

void Mapper::reset() {
  count = 0;
  lastEventTime = 0;
  runStartTime = 0;
  lastPredictedIndex = -1;
  for (int i = 0; i < MAX_WAYPOINTS; i++) {
    waypoints[i] = {EVENT_NONE, ACT_NONE, 0};
  }
}

void Mapper::mulaiRun() {
  runStartTime = millis();
  lastEventTime = runStartTime;  // baseline agar offset pertama = now - runStartTime
  lastPredictedIndex = -1;
}

void Mapper::recordEvent(EventType type, ActionType action) {
  if (count >= MAX_WAYPOINTS) return;

  unsigned long now = millis();
  // Offset selalu RELATIF terhadap lastEventTime (diinisialisasi = runStartTime
  // oleh mulaiRun()). Offset absolut (millis) tidak pernah sama dengan elapsed
  // di checkPrediction -> waypoint pertama tidak pernah terdeteksi.
  unsigned long offset = now - lastEventTime;

  waypoints[count] = {type, action, offset};
  count++;
  lastEventTime = now;
}

bool Mapper::checkPrediction(unsigned long currentTime, EventType &outType, ActionType &outAction) {
  if (count == 0) return false;

  // Baseline otomatis di panggilan pertama (jika user lupa memanggil mulaiRun).
  if (runStartTime == 0) {
    runStartTime = currentTime;
    lastPredictedIndex = -1;
  }

  // Waktu RELATIF sejak run dimulai — konsisten dengan timeOffset yang
  // direkam (offset relatif antar waypoint). currentTime absolut (millis())
  // tidak pernah sama dengan accumulatedTime -> prediksi tidak pernah fire.
  unsigned long elapsed = currentTime - runStartTime;

  // Prediksi fires sedikit SEBELUM waypoint (150ms) agar robot punya
  // waktu siapkan kecepatan/arah sebelum event benar-benar datang.
  const unsigned long PREDICT_LEAD_MS = 150;

  unsigned long accumulatedTime = 0;
  for (int i = 0; i < count; i++) {
    accumulatedTime += waypoints[i].timeOffset;

    // Window prediksi: [accumulatedTime - LEAD, accumulatedTime + 100)
    // Cuma sekali per waypoint (i > lastPredictedIndex).
    if (i > lastPredictedIndex &&
        elapsed >= accumulatedTime - PREDICT_LEAD_MS &&
        elapsed < accumulatedTime + 100) {
      lastPredictedIndex = i;
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
