#include "SafetyMonitor.h"
#include "DisplayUI.h"
#include "PlatformDrive.h"

void SafetyMonitor::checkHomedDeparture(
    float totalTravelMm,
    float remainingMm,
    bool isDepartureSwitchActive,
    const MotionFaultCodes& codes) {
  if ((totalTravelMm - remainingMm) > ERROR_TRAVEL_DISTANCE_MM) {
    if (isDepartureSwitchActive) {
      triggerFatalError(codes.stuckOnHomedDeparture);
    }
  }
}

void SafetyMonitor::checkUnhomedOrOverrun(
    float totalTravelMm,
    float traveledMm,
    bool remainingDistanceDone,
    bool isDepartureSwitchActive,
    const MotionFaultCodes& codes) {
  if (traveledMm > ERROR_TRAVEL_DISTANCE_MM) {
    if (isDepartureSwitchActive) {
      triggerFatalError(codes.stuckOnUnhomedDeparture);
    }
  }

  if (traveledMm > (totalTravelMm + codes.overrunMarginMm)) {
    triggerFatalError(codes.fullTravelOverrun);
  }

  if (remainingDistanceDone && (traveledMm > codes.overrunMarginMm)) {
    triggerFatalError(codes.homedEndOverrun);
  }
}

void SafetyMonitor::triggerFatalError(const char* errorCode) {
  platformDrive.disableMotor();
  platformDrive.stopTimer();
  displayUI.showStatus(errorCode);
  displayUI.playContinuousErrorTone();
  while (true) {
    // Halted until power cycle (recoverable ERROR_ST planned for Phase 2)
  }
}

