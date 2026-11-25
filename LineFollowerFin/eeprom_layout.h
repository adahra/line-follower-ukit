// eeprom_layout.h — Versioned EEPROM layout with compile-time checks.
// Single source of truth for all EEPROM addresses and layout.
// Uses static_assert to catch overflow/overlap at compile time.
#ifndef EEPROM_LAYOUT_H
#define EEPROM_LAYOUT_H

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
// EEPROM size for ATmega2560
static constexpr int EEPROM_TOTAL_SIZE = 4096;

// Current EEPROM format version
static constexpr uint8_t EEPROM_FMT_VERSION = 2;

// Magic bytes for validation
static constexpr uint8_t MAGIC_EEPROM_VALID = 123;
static constexpr uint8_t MAGIC_MAP_VALID = 77;
static constexpr uint8_t MAGIC_ADAPT_VALID = 66;
static constexpr uint8_t MAGIC_HEADING_VALID = 0x48;  // 'H'

// Compile-time address layout
// Keep all addresses as constexpr for static_assert
namespace EEPROM_ADDR {
  // Header (0-3)
  constexpr int EE_ADDR_VERSION = 0;
  constexpr int EE_ADDR_MAGIC = 1;
  constexpr int EE_ADDR_RESERVED_2 = 2;
  constexpr int EE_ADDR_RESERVED_3 = 3;

  // PID Gains (4-15) - 4 bytes each (float)
  constexpr int EE_ADDR_KP = 4;
  constexpr int EE_ADDR_KI = 8;
  constexpr int EE_ADDR_KD = 12;

  // Speed (16-19) - 2 bytes (int16) + 2 reserved
  constexpr int EE_ADDR_SPEED_MAX = 16;
  constexpr int EE_ADDR_SPEED_MIN = 18;  // NEW: persist min speed

  // Thresholds (20-23) - 2 bytes each (uint16)
  constexpr int EE_ADDR_AMBANG_KIRI = 20;
  constexpr int EE_ADDR_AMBANG_KANAN = 22;

  // Waypoint Map (24-279) - 256 bytes max
  // Magic(1) + Count(2) + Waypoint[30] * 8 = 243 bytes
  constexpr int EE_ADDR_MAP_START = 24;
  constexpr int EE_ADDR_MAP_MAGIC = 24;
  constexpr int EE_ADDR_MAP_COUNT = 25;
  constexpr int EE_ADDR_MAP_WAYPOINTS = 27;
  constexpr int EE_ADDR_MAP_MAX_SIZE = 256;  // up to address 279

  // Heading Correction (280-295) - 16 bytes
  constexpr int EE_ADDR_HEADING_START = 280;
  constexpr int EE_ADDR_HEADING_MAGIC = 280;
  constexpr int EE_ADDR_HEADING_GAIN = 281;    // float (4 bytes)
  constexpr int EE_ADDR_HEADING_TARGET = 285;  // float (4 bytes)
  constexpr int EE_ADDR_HEADING_BIAS = 289;    // float (4 bytes)
  constexpr int EE_ADDR_HEADING_CALIBRATED = 293;  // bool (1 byte)
  constexpr int EE_ADDR_HEADING_END = 295;

  // Adaptive Threshold (296-301) - 6 bytes
  constexpr int EE_ADDR_ADAPT_START = 296;
  constexpr int EE_ADDR_ADAPT_MAGIC = 296;
  constexpr int EE_ADDR_ADAPT_AMBANG_KIRI = 297;    // uint16 (2 bytes)
  constexpr int EE_ADDR_ADAPT_AMBANG_KANAN = 299;   // uint16 (2 bytes)
  constexpr int EE_ADDR_ADAPT_ENABLED = 301;        // bool (1 byte)
  constexpr int EE_ADDR_ADAPT_END = 301;

  // Future expansion (302-4095)
  constexpr int EE_ADDR_NEXT_FREE = 302;
}

// Struct matching EEPROM layout exactly
#pragma pack(push, 1)
struct EEPROMData {
  // Header
  uint8_t version = EEPROM_FMT_VERSION;
  uint8_t magic = MAGIC_EEPROM_VALID;
  uint8_t reserved[2] = {0, 0};

  // PID Gains
  float kp = 7.0;
  float ki = 0.05;
  float kd = 2.2;

  // Speed
  int16_t speedMax = 100;
  int16_t speedMin = 30;

  // Thresholds
  uint16_t ambangKiri = 8;
  uint16_t ambangKanan = 8;

  // Waypoint Map (simplified - actual storage uses separate functions)
  // Map stored at MAP_START with its own magic
  uint8_t mapMagic = 0;
  uint16_t mapCount = 0;
  // Waypoints follow at MAP_WAYPOINTS...

  // Heading Correction
  uint8_t headingMagic = 0;
  float headingGain = 0.8f;  // Use literal to avoid macro conflict with config.h
  float headingTarget = 0.0;
  float headingBias = 0.0;
  bool headingCalibrated = false;

  // Adaptive Threshold
  uint8_t adaptMagic = 0;
  uint16_t adaptAmbangKiri = 0;
  uint16_t adaptAmbangKanan = 0;
  bool adaptEnabled = false;
};
#pragma pack(pop)

// Compile-time checks
static_assert(sizeof(EEPROMData) <= EEPROM_ADDR::EE_ADDR_NEXT_FREE,
  "EEPROMData struct exceeds allocated space");

static_assert(EEPROM_ADDR::EE_ADDR_KP + 4 <= EEPROM_ADDR::EE_ADDR_KI,
  "KP overlaps KI");
static_assert(EEPROM_ADDR::EE_ADDR_KI + 4 <= EEPROM_ADDR::EE_ADDR_KD,
  "KI overlaps KD");
static_assert(EEPROM_ADDR::EE_ADDR_KD + 4 <= EEPROM_ADDR::EE_ADDR_SPEED_MAX,
  "KD overlaps SPEED_MAX");
static_assert(EEPROM_ADDR::EE_ADDR_SPEED_MAX + 2 <= EEPROM_ADDR::EE_ADDR_SPEED_MIN,
  "SPEED_MAX overlaps SPEED_MIN");
static_assert(EEPROM_ADDR::EE_ADDR_SPEED_MIN + 2 <= EEPROM_ADDR::EE_ADDR_AMBANG_KIRI,
  "SPEED_MIN overlaps AMBANG_KIRI");
static_assert(EEPROM_ADDR::EE_ADDR_AMBANG_KIRI + 2 <= EEPROM_ADDR::EE_ADDR_AMBANG_KANAN,
  "AMBANG_KIRI overlaps AMBANG_KANAN");
static_assert(EEPROM_ADDR::EE_ADDR_AMBANG_KANAN + 2 <= EEPROM_ADDR::EE_ADDR_MAP_START,
  "AMBANG_KANAN overlaps MAP_START");
static_assert(EEPROM_ADDR::EE_ADDR_MAP_START + EEPROM_ADDR::EE_ADDR_MAP_MAX_SIZE <= EEPROM_ADDR::EE_ADDR_HEADING_START,
  "MAP overlaps HEADING");
static_assert(EEPROM_ADDR::EE_ADDR_HEADING_START + 16 <= EEPROM_ADDR::EE_ADDR_ADAPT_START,
  "HEADING overlaps ADAPT");
static_assert(EEPROM_ADDR::EE_ADDR_ADAPT_START + 6 <= EEPROM_ADDR::EE_ADDR_NEXT_FREE,
  "ADAPT exceeds NEXT_FREE");
static_assert(EEPROM_ADDR::EE_ADDR_NEXT_FREE <= EEPROM_TOTAL_SIZE,
  "EEPROM layout exceeds 4KB");

// Helper: write struct field only if changed (like eepromPutHemat)
template <typename T>
inline void eepromWriteIfChanged(int addr, const T &value) {
  T current;
  EEPROM.get(addr, current);
  if (memcmp(&current, &value, sizeof(T)) != 0) {
    EEPROM.put(addr, value);
  }
}

// Initialize EEPROM with defaults if invalid
inline void eepromInit() {
  uint8_t magic = EEPROM.read(EEPROM_ADDR::EE_ADDR_MAGIC);
  uint8_t version = EEPROM.read(EEPROM_ADDR::EE_ADDR_VERSION);

  if (magic != MAGIC_EEPROM_VALID || version != EEPROM_FMT_VERSION) {
    EEPROMData defaults;
    EEPROM.put(0, defaults);  // Write header + PID + speed + thresholds

    // Initialize map magic to invalid
    EEPROM.update(EEPROM_ADDR::EE_ADDR_MAP_MAGIC, 0);
    // Initialize heading magic to invalid
    EEPROM.update(EEPROM_ADDR::EE_ADDR_HEADING_MAGIC, 0);
    // Initialize adaptive magic to invalid
    EEPROM.update(EEPROM_ADDR::EE_ADDR_ADAPT_MAGIC, 0);
  }
}

// Read PID gains from EEPROM (with bounds checking)
inline void eepromReadPID(float &kp, float &ki, float &kd) {
  if (EEPROM.read(EEPROM_ADDR::EE_ADDR_MAGIC) == MAGIC_EEPROM_VALID) {
    EEPROM.get(EEPROM_ADDR::EE_ADDR_KP, kp);
    EEPROM.get(EEPROM_ADDR::EE_ADDR_KI, ki);
    EEPROM.get(EEPROM_ADDR::EE_ADDR_KD, kd);
    // Clamp to reasonable bounds
    kp = constrain(kp, 0.0f, 50.0f);
    ki = constrain(ki, 0.0f, 5.0f);
    kd = constrain(kd, 0.0f, 50.0f);
  }
}

// Write PID gains to EEPROM
inline void eepromWritePID(float kp, float ki, float kd) {
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_KP, kp);
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_KI, ki);
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_KD, kd);
}

// Read speed settings
inline void eepromReadSpeed(int16_t &maxSpeed, int16_t &minSpeed) {
  if (EEPROM.read(EEPROM_ADDR::EE_ADDR_MAGIC) == MAGIC_EEPROM_VALID) {
    EEPROM.get(EEPROM_ADDR::EE_ADDR_SPEED_MAX, maxSpeed);
    EEPROM.get(EEPROM_ADDR::EE_ADDR_SPEED_MIN, minSpeed);
    maxSpeed = constrain(maxSpeed, 10, 150);
    minSpeed = constrain(minSpeed, 10, 150);
  }
}

// Write speed settings
inline void eepromWriteSpeed(int16_t maxSpeed, int16_t minSpeed) {
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_SPEED_MAX, maxSpeed);
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_SPEED_MIN, minSpeed);
}

// Read thresholds
inline void eepromReadThresholds(int &ambangKiri, int &ambangKanan) {
  if (EEPROM.read(EEPROM_ADDR::EE_ADDR_MAGIC) == MAGIC_EEPROM_VALID) {
    uint16_t kiri, kanan;
    EEPROM.get(EEPROM_ADDR::EE_ADDR_AMBANG_KIRI, kiri);
    EEPROM.get(EEPROM_ADDR::EE_ADDR_AMBANG_KANAN, kanan);
    ambangKiri = constrain((int)kiri, 1, 20);
    ambangKanan = constrain((int)kanan, 1, 20);
  }
}

// Write thresholds
inline void eepromWriteThresholds(int ambangKiri, int ambangKanan) {
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_AMBANG_KIRI, (uint16_t)ambangKiri);
  eepromWriteIfChanged(EEPROM_ADDR::EE_ADDR_AMBANG_KANAN, (uint16_t)ambangKanan);
}

#endif // EEPROM_LAYOUT_H