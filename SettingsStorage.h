#ifndef SETTINGS_STORAGE_H
#define SETTINGS_STORAGE_H

#include "Config.h"

// Stored in EEPROM at EP_ADDR_FAULT_RECORD
struct FaultRecord {
  uint16_t magic;         // 0x464C ("FL") indicates valid record
  uint8_t  code;          // FaultCode enum
  uint8_t  state;         // PlatformState when fault happened
  float    travelMm;      // Travel distance when fault occurred
  uint8_t  acknowledged;  // 0 = unacknowledged (causes boot alarm), 1 = acknowledged
  uint8_t  reserved;      // Alignment padding
  uint16_t checksum;      // Simple checksum for data validation
};

class SettingsStorage {
public:
  // Reads calibrated travel distance from EEPROM, falling back to DEFAULT_TRAVEL_DISTANCE_MM
  // if uninitialized (NaN) or outside +/-10% sanity bounds.
  static float loadTravelDistance();

  // Saves calibrated travel distance to EEPROM only if the value changed (to avoid wear).
  static void saveTravelDistance(float travelDistanceMm);

  // Fault history EEPROM storage & boot diagnostics
  static FaultRecord loadFaultRecord();
  static void saveFaultRecord(FaultCode code, PlatformState state, float travelMm);
  static void acknowledgeFaultRecord();
  static void clearFaultRecord();

  // Converts FaultCode to 4-character display mnemonic (e.g. "StSt", "ovFd", "nonE")
  static const char* getFaultMnemonic(FaultCode code);
  // Returns human-readable description for Serial logs
  static const char* getFaultDescription(FaultCode code);
  // Returns legacy error code string (e.g. "Err1", "Err3") for backward compatibility
  static const char* getLegacyErrorCode(FaultCode code, PlatformState state);

  // Check if an unacknowledged fault exists from previous run
  static bool hasUnacknowledgedFault();
};

#endif // SETTINGS_STORAGE_H
