#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "Config.h"
#include "SafetyMonitor.h"

// Sub-phases for precision back-off homing
enum HomingPhase : uint8_t {
  HOMING_PHASE_CLEAR_FWD = 0, // Step forward at 160 RPM until Start switch goes HIGH
  HOMING_PHASE_SETTLE,        // Settle briefly
  HOMING_PHASE_CREEP_REV      // Reverse at -160 RPM until switch trips LOW
};

class StateMachine {
public:
  StateMachine();

  // Loads EEPROM settings and initializes state variables.
  void begin();

  // Executes one iteration of the active platform state.
  void update();

  // Transitions immediately to ERROR_ST upon safety fault
  void enterErrorState(FaultCode code);

  PlatformState getState() const { return state_; }
  FaultCode     getActiveFault() const { return activeFault_; }

private:
  // Full variable reset (used at power-up and when entering Setup via 4s Reset hold).
  void resetVariables();

  // State transition helpers
  void enterSetupState();
  void enterStopState();
  void enterRunningFromStop(bool performHomingCheck);
  void enterFastForwardFromStop();
  void enterReturnFromRunning(bool convertRemainingDistance);
  void enterReturnFromStop();
  void enterReturnFromWait();
  void enterWaitReturnState();
  void enterHomingSequence();
  void enterCalibrationState();
  void executeParkSequence();
  void startReturnMotion();
  void finishCalibrationMeasurement(float measuredMm);

  // Per-state update handlers
  void updateSetupState();
  void updateStopState();
  void updateRunningState();
  void updateFastForwardState();
  void updateReturnState();
  void updateWaitReturnState();
  void updateErrorState();
  void updateHomingState();
  void updateCalibrationState();

  // Shared distance, safety, and progress-display handler for Fast-Forward and Return
  void updateFastMotionProgress(
      float effectivePositiveRpm,
      bool invertPercentage,
      bool isDepartureSwitchActive,
      FaultCode departureFault,
      FaultCode overrunFault,
      float overrunMarginMm);

  // Closed-loop adaptive calibration helpers
  void evaluateRunForAdaptiveCalibration();
  void applyAdaptiveCalibrationDelta(float deltaMm);

  // Motion distance integration and departure-relative position helpers
  void  integrateMotionStep(float effectivePositiveRpm);
  float getDistanceFromDepartureMm() const;

  PlatformState state_;
  SetupMode     setupMode_;
  ReturnPolicy  returnPolicy_;

  float totalTravelDistanceMm_;
  float remainingDistanceMm_;
  float travelDistanceMm_;
  bool  remainingDistanceDone_;
  bool  isHomed_;

  // Adaptive auto-calibration tracking
  bool  cleanRunEligible_;
  float savedTravelDistanceMm_;
  bool  returnShowCalInfo_;
  bool  deviationAlertPending_;
  unsigned long deviationBlinkMs_;
  bool          deviationBlinkToggle_;

  bool printTrackingDebug_;
  int  easterEggCounter_;

  // Error state variables
  FaultCode     activeFault_;
  unsigned long errorBlinkMs_;
  unsigned long errorChirpMs_;
  bool          errorBlinkToggle_;
  bool          errorChirpMuted_;
  unsigned long resetHoldInErrorMs_;

  // Homing sequence variables
  HomingPhase   homingPhase_;
  float         homingTraveledMm_;
  float         homingClearStartMm_;
  unsigned long homingSettleStartMs_;

  // Calibration, Setup & Error Reset hold state variables
  bool          calibrationFinished_;
  float         measuredCalibrationMm_;
  unsigned long calibrationBlinkMs_;
  bool          calibrationBlinkToggle_;
  unsigned long resetHoldInCalMs_;
  bool          resetHoldTiming_;
  bool          resetHandledInState_;

  unsigned long startMs_;
  unsigned long elapsedMs_;
  unsigned long timeoutMs_;
  unsigned long setupCountdownMs_;
};

extern StateMachine stateMachine;

#endif // STATE_MACHINE_H
