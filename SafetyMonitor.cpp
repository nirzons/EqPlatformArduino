#include "SafetyMonitor.h"
#include "InputManager.h"
#include "StateMachine.h"

bool SafetyMonitor::checkSwitchShort() {
  if (inputManager.checkBothSwitchesActiveDebounced()) {
    stateMachine.enterErrorState(FAULT_SWITCH_SHORT);
    return true;
  }
  return false;
}

bool SafetyMonitor::checkUnexpectedStartSwitch(float distanceFromStartMm) {
  if (distanceFromStartMm > (ERROR_TRAVEL_DISTANCE_MM * 2.0f)) {
    if (inputManager.isStartSwitchDebouncedActive()) {
      stateMachine.enterErrorState(FAULT_WRONG_DIRECTION);
      return true;
    }
  }
  return false;
}

bool SafetyMonitor::checkUnexpectedStopSwitch(float distanceFromStopMm) {
  if (distanceFromStopMm > (ERROR_TRAVEL_DISTANCE_MM * 2.0f)) {
    if (inputManager.isStopSwitchDebouncedActive()) {
      stateMachine.enterErrorState(FAULT_WRONG_DIRECTION);
      return true;
    }
  }
  return false;
}

bool SafetyMonitor::checkHomedDeparture(
    float totalTravelMm,
    float remainingMm,
    bool isDepartureSwitchActive,
    FaultCode departureFault) {
  if ((totalTravelMm - remainingMm) > ERROR_TRAVEL_DISTANCE_MM) {
    if (isDepartureSwitchActive) {
      stateMachine.enterErrorState(departureFault);
      return true;
    }
  }
  return false;
}

bool SafetyMonitor::checkUnhomedOrOverrun(
    float totalTravelMm,
    float traveledMm,
    bool remainingDistanceDone,
    bool isDepartureSwitchActive,
    FaultCode departureFault,
    FaultCode overrunFault,
    float overrunMarginMm) {
  if (!remainingDistanceDone && (traveledMm > ERROR_TRAVEL_DISTANCE_MM)) {
    if (isDepartureSwitchActive) {
      stateMachine.enterErrorState(departureFault);
      return true;
    }
  }

  if (traveledMm > (totalTravelMm + overrunMarginMm)) {
    stateMachine.enterErrorState(overrunFault);
    return true;
  }

  if (remainingDistanceDone && (traveledMm > overrunMarginMm)) {
    stateMachine.enterErrorState(overrunFault);
    return true;
  }

  return false;
}
