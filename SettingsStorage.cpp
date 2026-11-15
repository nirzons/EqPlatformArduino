#include "SettingsStorage.h"
#include <EEPROM.h>
#include <math.h>

namespace {
constexpr uint16_t FAULT_RECORD_MAGIC = 0x464C; // "FL"

uint16_t computeChecksum(const FaultRecord& rec) {
  uint16_t sum = rec.magic + rec.code + rec.state + rec.acknowledged;
  const uint8_t* pFloat = reinterpret_cast<const uint8_t*>(&rec.travelMm);
  for (size_t i = 0; i < sizeof(float); i++) {
    sum += pFloat[i];
  }
  return sum;
}
} // namespace

float SettingsStorage::loadTravelDistance() {
  Serial.println(F("Read from EEPROM"));

  float storedDistance = 0.0f;
  EEPROM.get(EP_ADDR_TRAVEL_DISTANCE, storedDistance);

  Serial.print(F("total_travel_distance: "));
  Serial.println(storedDistance, 1);

  const float minValid = DEFAULT_TRAVEL_DISTANCE_MM * TRAVEL_DISTANCE_MIN_RATIO;
  const float maxValid = DEFAULT_TRAVEL_DISTANCE_MM * TRAVEL_DISTANCE_MAX_RATIO;

  if (isnan(storedDistance) || (storedDistance < minValid) || (storedDistance > maxValid)) {
    storedDistance = DEFAULT_TRAVEL_DISTANCE_MM;
    Serial.print(F("total_travel_distance set to default: "));
    Serial.println(storedDistance, 1);
  }

  return storedDistance;
}

void SettingsStorage::saveTravelDistance(float travelDistanceMm) {
  Serial.println(F("Write to EEPROM"));

  float currentStored = 0.0f;
  EEPROM.get(EP_ADDR_TRAVEL_DISTANCE, currentStored);

  if (currentStored != travelDistanceMm) {
    Serial.print(F("set total_travel_distance to: "));
    Serial.println(travelDistanceMm, 1);
    EEPROM.put(EP_ADDR_TRAVEL_DISTANCE, travelDistanceMm);
  }
}

FaultRecord SettingsStorage::loadFaultRecord() {
  FaultRecord record;
  EEPROM.get(EP_ADDR_FAULT_RECORD, record);

  if (record.magic != FAULT_RECORD_MAGIC || record.checksum != computeChecksum(record)) {
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
  record.checksum     = computeChecksum(record);

  EEPROM.put(EP_ADDR_FAULT_RECORD, record);

  Serial.print(F("EEPROM saved fault: "));
  Serial.print(getFaultMnemonic(code));
  Serial.print(F(" ("));
  Serial.print(getFaultDescription(code));
  Serial.print(F(") at "));
  Serial.print(travelMm, 1);
  Serial.println(F(" mm"));
}

void SettingsStorage::acknowledgeFaultRecord() {
  FaultRecord record = loadFaultRecord();
  if (record.magic == FAULT_RECORD_MAGIC && record.acknowledged == 0) {
    record.acknowledged = 1;
    record.checksum     = computeChecksum(record);
    EEPROM.put(EP_ADDR_FAULT_RECORD, record);
    Serial.println(F("Fault acknowledged in EEPROM"));
  }
}

void SettingsStorage::clearFaultRecord() {
  FaultRecord emptyRecord = { 0, FAULT_NONE, STOP_ST, 0.0f, 1, 0, 0 };
  EEPROM.put(EP_ADDR_FAULT_RECORD, emptyRecord);
  Serial.println(F("Fault history cleared in EEPROM"));
}

bool SettingsStorage::hasUnacknowledgedFault() {
  FaultRecord record = loadFaultRecord();
  return (record.magic == FAULT_RECORD_MAGIC && record.code != FAULT_NONE && record.acknowledged == 0);
}

const char* SettingsStorage::getFaultMnemonic(FaultCode code) {
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

const char* SettingsStorage::getFaultDescription(FaultCode code) {
  switch (code) {
    case FAULT_START_SWITCH_STUCK: return "Start Switch Stuck";
    case FAULT_STOP_SWITCH_STUCK:  return "Stop Switch Stuck";
    case FAULT_OVERTRAVEL_FORWARD: return "Overtravel Forward (No Stop Switch)";
    case FAULT_OVERTRAVEL_RETURN:  return "Overtravel Return (No Start Switch)";
    case FAULT_SWITCH_SHORT:       return "Both Limit Switches Active (Short/Fault)";
    case FAULT_WRONG_DIRECTION:    return "Wrong Switch Triggered for Direction";
    default:                       return "No Fault";
  }
}

const char* SettingsStorage::getLegacyErrorCode(FaultCode code, PlatformState state) {
  switch (code) {
    case FAULT_START_SWITCH_STUCK:
      if (state == RUNNING_ST) return "Err1/Err2";
      if (state == FAST_FORWARD_ST) return "Err4/Err5";
      return "Err1";
    case FAULT_STOP_SWITCH_STUCK:
      return "Err9/Er11";
    case FAULT_OVERTRAVEL_FORWARD:
      if (state == RUNNING_ST) return "Err3/Err8";
      if (state == FAST_FORWARD_ST) return "Err6/Err7";
      return "Err3";
    case FAULT_OVERTRAVEL_RETURN:
      return "Err0/Er12";
    case FAULT_SWITCH_SHORT:
      return "SwSh";
    case FAULT_WRONG_DIRECTION:
      return "dirE";
    default:
      return "OK";
  }
}
