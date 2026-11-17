#include "SettingsStorage.h"
#include <EEPROM.h>
#include <math.h>

namespace {
constexpr uint16_t FAULT_RECORD_MAGIC = 0x464C; // "FL"
constexpr uint16_t PARK_RECORD_MAGIC  = 0x4551; // "EQ"

uint16_t computeFaultChecksum(const FaultRecord& rec) {
  uint16_t sum = rec.magic + rec.code + rec.state + rec.acknowledged;
  const uint8_t* pFloat = reinterpret_cast<const uint8_t*>(&rec.travelMm);
  for (size_t i = 0; i < sizeof(float); i++) {
    sum += pFloat[i];
  }
  return sum;
}

uint16_t computeParkChecksum(const ParkRecord& rec) {
  uint16_t sum = rec.magic + rec.trackingRate;
  const uint8_t* pFloat = reinterpret_cast<const uint8_t*>(&rec.remainingMm);
  for (size_t i = 0; i < sizeof(float); i++) {
    sum += pFloat[i];
  }
  return sum;
}

bool isParkRecordValid(const ParkRecord& rec) {
  if (rec.magic != PARK_RECORD_MAGIC || rec.checksum != computeParkChecksum(rec)) {
    return false;
  }
  const float maxValid = DEFAULT_TRAVEL_DISTANCE_MM * TRAVEL_DISTANCE_MAX_RATIO;
  if (isnan(rec.remainingMm) || rec.remainingMm <= 0.0f || rec.remainingMm > maxValid) {
    return false;
  }
#ifdef SUN_RATE_SUPPORT
  if (rec.trackingRate > static_cast<uint8_t>(SUN_RATE)) {
    return false;
  }
#else
  if (rec.trackingRate > static_cast<uint8_t>(MOON_RATE)) {
    return false;
  }
#endif
  return true;
}
} // namespace

float SettingsStorage::loadTravelDistance() {
  float storedDistance = 0.0f;
  EEPROM.get(EP_ADDR_TRAVEL_DISTANCE, storedDistance);

  Serial.print(F("Dist: "));
  Serial.println(storedDistance, 1);

  const float minValid = DEFAULT_TRAVEL_DISTANCE_MM * TRAVEL_DISTANCE_MIN_RATIO;
  const float maxValid = DEFAULT_TRAVEL_DISTANCE_MM * TRAVEL_DISTANCE_MAX_RATIO;

  if (isnan(storedDistance) || (storedDistance < minValid) || (storedDistance > maxValid)) {
    storedDistance = DEFAULT_TRAVEL_DISTANCE_MM;
    Serial.print(F("Default dist: "));
    Serial.println(storedDistance, 1);
  }

  return storedDistance;
}

__attribute__((noinline)) void SettingsStorage::saveTravelDistance(float travelDistanceMm) {
  float currentStored = 0.0f;
  EEPROM.get(EP_ADDR_TRAVEL_DISTANCE, currentStored);

  if (currentStored != travelDistanceMm) {
    Serial.print(F("Save dist: "));
    Serial.println(travelDistanceMm, 1);
    EEPROM.put(EP_ADDR_TRAVEL_DISTANCE, travelDistanceMm);
  }
}

__attribute__((noinline)) FaultRecord SettingsStorage::loadFaultRecord() {
  FaultRecord record;
  EEPROM.get(EP_ADDR_FAULT_RECORD, record);

  if (record.magic != FAULT_RECORD_MAGIC ||
      record.checksum != computeFaultChecksum(record) ||
      record.code > static_cast<uint8_t>(FAULT_WRONG_DIRECTION) ||
      isnan(record.travelMm)) {
    record.magic        = 0;
    record.code         = FAULT_NONE;
    record.state        = STOP_ST;
    record.travelMm     = 0.0f;
    record.acknowledged = 1;
    record.reserved     = 0;
    record.checksum     = 0;
  }
  return record;
}

void SettingsStorage::saveFaultRecord(FaultCode code, PlatformState state, float travelMm) {
  FaultRecord record;
  record.magic        = FAULT_RECORD_MAGIC;
  record.code         = static_cast<uint8_t>(code);
  record.state        = static_cast<uint8_t>(state);
  record.travelMm     = travelMm;
  record.acknowledged = 0; // Unacknowledged
  record.reserved     = 0;
  record.checksum     = computeFaultChecksum(record);

  EEPROM.put(EP_ADDR_FAULT_RECORD, record);
}

void SettingsStorage::acknowledgeFaultRecord() {
  FaultRecord record = loadFaultRecord();
  if (record.magic == FAULT_RECORD_MAGIC && record.acknowledged == 0) {
    record.acknowledged = 1;
    record.checksum     = computeFaultChecksum(record);
    EEPROM.put(EP_ADDR_FAULT_RECORD, record);
  }
}

void SettingsStorage::clearFaultRecord() {
  FaultRecord emptyRecord = { 0, FAULT_NONE, STOP_ST, 0.0f, 1, 0, 0 };
  EEPROM.put(EP_ADDR_FAULT_RECORD, emptyRecord);
}

bool SettingsStorage::hasValidParkRecord() {
  ParkRecord record;
  EEPROM.get(EP_ADDR_PARK_RECORD, record);
  return isParkRecordValid(record);
}

ParkRecord SettingsStorage::loadParkRecord() {
  ParkRecord record;
  EEPROM.get(EP_ADDR_PARK_RECORD, record);
  if (!isParkRecordValid(record)) {
    record.magic        = 0;
    record.remainingMm  = 0.0f;
    record.trackingRate = SIDEREAL_RATE;
    record.reserved     = 0;
    record.checksum     = 0;
  }
  return record;
}

void SettingsStorage::saveParkRecord(float remainingMm, TrackingRate rate) {
  ParkRecord record;
  record.magic        = PARK_RECORD_MAGIC;
  record.remainingMm  = remainingMm;
  record.trackingRate = static_cast<uint8_t>(rate);
  record.reserved     = 0;
  record.checksum     = computeParkChecksum(record);

  EEPROM.put(EP_ADDR_PARK_RECORD, record);
  Serial.println(F("Park saved"));
}

void SettingsStorage::clearParkRecord() {
  ParkRecord emptyRecord = { 0, 0.0f, SIDEREAL_RATE, 0, 0 };
  EEPROM.put(EP_ADDR_PARK_RECORD, emptyRecord);
}

__attribute__((noinline)) const char* SettingsStorage::getFaultMnemonic(FaultCode code) {
  switch (code) {
    case FAULT_START_SWITCH_STUCK: return "StSt";
    case FAULT_STOP_SWITCH_STUCK:  return "StSP";
    case FAULT_OVERTRAVEL_FORWARD: return "ovFd";
    case FAULT_OVERTRAVEL_RETURN:  return "ovRt";
    case FAULT_SWITCH_SHORT:       return "SwSh";
    case FAULT_WRONG_DIRECTION:    return "dirE";
    default:                       return "nonE";
  }
}
