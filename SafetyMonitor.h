#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include "Config.h"

class SafetyMonitor {
public:
  // Hardware sanity check: checks if both switches are reading LOW simultaneously
  // (debounced over SWITCH_DEBOUNCE_SAMPLES to prevent noise spikes from triggering).
  static bool checkSwitchShort();

  // Direction sanity checks (debounced):
  // In forward travel (Tracking / FF), once departed from Start switch (> 5.4 mm), Start switch shouldn't trigger.
  static bool checkUnexpectedStartSwitch(float distanceFromStartMm);
  // In return travel, once departed from Stop switch (> 5.4 mm), Stop switch shouldn't trigger.
  static bool checkUnexpectedStopSwitch(float distanceFromStopMm);

  // Validates that the departure limit switch cleared after moving ERROR_TRAVEL_DISTANCE_MM
  // during a homed move (when remainingDistanceMm started > 0).
  // Returns true if a fault was triggered.
  static bool checkHomedDeparture(
      float totalTravelMm,
      float remainingMm,
      bool isDepartureSwitchActive,
      FaultCode departureFault);

  // Validates departure switch release and end-of-travel overrun limits when moving
  // without remaining distance (unhomed start or after remainingDistance reached 0).
  // Returns true if a fault was triggered.
  static bool checkUnhomedOrOverrun(
      float totalTravelMm,
      float traveledMm,
      bool remainingDistanceDone,
      bool isDepartureSwitchActive,
      FaultCode departureFault,
      FaultCode overrunFault,
      float overrunMarginMm);
};

#endif // SAFETY_MONITOR_H
