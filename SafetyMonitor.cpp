#include "SafetyMonitor.h"
#include "InputManager.h"
#include "StateMachine.h"

__attribute__((noinline)) bool SafetyMonitor::checkSwitchShort() {
  if (inputManager.checkBothSwitchesActiveDebounced()) {
    stateMachine.enterErrorState(FAULT_SWITCH_SHORT);
    return true;
  }
  return false;
}

__attribute__((noinline)) bool SafetyMonitor::checkUnexpectedStartSwitch(float distanceFromStartMm) {
  const bool active = inputManager.isStartSwitchDebouncedActive();
  if (active && (distanceFromStartMm > (ERROR_TRAVEL_DISTANCE_MM * 2.0f))) {
    stateMachine.enterErrorState(FAULT_WRONG_DIRECTION);
    return true;
  }
  return false;
}

__attribute__((noinline)) bool SafetyMonitor::checkUnexpectedStopSwitch(float distanceFromStopMm) {
  const bool active = inputManager.isStopSwitchDebouncedActive();
  if (active && (distanceFromStopMm > (ERROR_TRAVEL_DISTANCE_MM * 2.0f))) {
    stateMachine.enterErrorState(FAULT_WRONG_DIRECTION);
    return true;
  }
  return false;
}

__attribute__((noinline)) bool SafetyMonitor::checkHomedDeparture(
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

__attribute__((noinline)) bool SafetyMonitor::checkUnhomedOrOverrun(
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
