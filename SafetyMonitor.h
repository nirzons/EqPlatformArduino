#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include "Config.h"

struct MotionFaultCodes {
  const char* stuckOnHomedDeparture;   // Departure limit switch still active when remainingDistance > 0
  const char* stuckOnUnhomedDeparture; // Departure limit switch still active when remainingDistance == 0
  const char* fullTravelOverrun;       // Exceeded totalTravelDistance + overrunMarginMm
  const char* homedEndOverrun;         // Exceeded overrunMarginMm after remainingDistance reached 0
  float       overrunMarginMm;         // ERROR_TRAVEL_DISTANCE_MM or END_ERROR_TRAVEL_DISTANCE_MM
};

class SafetyMonitor {
public:
  // Validates that the departure limit switch cleared after moving ERROR_TRAVEL_DISTANCE_MM
  // during a homed move (when remainingDistanceMm started > 0).
  static void checkHomedDeparture(
      float totalTravelMm,
      float remainingMm,
      bool isDepartureSwitchActive,
      const MotionFaultCodes& codes);

  // Validates departure switch release and end-of-travel overrun limits when moving
  // without remaining distance (unhomed start or after remainingDistance reached 0).
  static void checkUnhomedOrOverrun(
      float totalTravelMm,
      float traveledMm,
      bool remainingDistanceDone,
      bool isDepartureSwitchActive,
      const MotionFaultCodes& codes);

  // Immediately disables motor & Timer1, displays fault message, sounds alarm, and halts.
  static void triggerFatalError(const char* errorCode);
};

#endif // SAFETY_MONITOR_H

