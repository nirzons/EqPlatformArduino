#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "Config.h"
#include "SafetyMonitor.h"

class StateMachine {
public:
  StateMachine();

  // Loads EEPROM settings and initializes state variables.
  void begin();

  // Executes one iteration of the active platform state and global Reset hold check.
  void update();

  PlatformState getState() const { return state_; }

private:
  // Full variable reset (used at power-up and when entering Setup via 4s Reset hold).
  void resetVariables();

  // State transition helpers
  void enterSetupState();
  void enterStopState();
  void enterRunningFromStop();
  void enterFastForwardFromRunning();
  void enterReturnFromRunning(bool convertRemainingDistance);
  void enterReturnFromWait();
  void enterWaitReturnState(bool restoreVelocitySnapshot);

  // Per-state update handlers
  void updateSetupState();
  void updateStopState();
  void updateRunningState();
  void updateFastForwardState();
  void updateReturnState();
  void updateWaitReturnState();

  // Shared distance, safety, and progress-display handler for Fast-Forward and Return
  void updateFastMotionProgress(
      float effectivePositiveRpm,
      bool invertPercentage,
      bool isDepartureSwitchActive,
      const MotionFaultCodes& faultCodes);

  PlatformState state_;
  SetupMode     setupMode_;
  ReturnPolicy  returnPolicy_;

  float totalTravelDistanceMm_;
  float remainingDistanceMm_;
  float travelDistanceMm_;
  bool  remainingDistanceDone_;

  bool printTrackingDebug_;
  bool resetButtonHandled_;
  int  easterEggCounter_;

  unsigned long startMs_;
  unsigned long elapsedMs_;
  unsigned long timeoutMs_;
  unsigned long setupCountdownMs_;
  unsigned long resetHoldStartMs_;
  unsigned long fastForwardHoldMs_;
  unsigned long fastBackwardHoldMs_;
};

extern StateMachine stateMachine;

#endif // STATE_MACHINE_H

