#ifndef SETTINGS_STORAGE_H
#define SETTINGS_STORAGE_H

#include "Config.h"

class SettingsStorage {
public:
  // Reads calibrated travel distance from EEPROM, falling back to DEFAULT_TRAVEL_DISTANCE_MM
  // if uninitialized (NaN) or outside +/-10% sanity bounds.
  static float loadTravelDistance();

  // Saves calibrated travel distance to EEPROM only if the value changed (to avoid wear).
  static void saveTravelDistance(float travelDistanceMm);
};

#endif // SETTINGS_STORAGE_H

