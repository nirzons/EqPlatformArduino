#include "SettingsStorage.h"
#include <EEPROM.h>
#include <math.h>

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

  for (int n = 0; n < 10; n++) {
    byte b = 0;
    EEPROM.get(n, b);
    Serial.print(n);
    Serial.print(F(": "));
    Serial.println(b);
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

