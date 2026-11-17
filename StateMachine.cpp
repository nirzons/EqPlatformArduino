#include "StateMachine.h"
#include "DisplayUI.h"
#include "InputManager.h"
#include "PlatformDrive.h"
#include "SettingsStorage.h"

StateMachine stateMachine;

StateMachine::StateMachine()
    : state_(INITIAL_PLATFORM_STATE),
      setupMode_(RETURN_POLICY_M),
      returnPolicy_(DEFAULT_RETURN_POLICY),
      totalTravelDistanceMm_(DEFAULT_TRAVEL_DISTANCE_MM),
      remainingDistanceMm_(0.0f),
      travelDistanceMm_(0.0f),
      remainingDistanceDone_(false),
      isHomed_(false),
      cleanRunEligible_(false),
      savedTravelDistanceMm_(DEFAULT_TRAVEL_DISTANCE_MM),
      returnShowCalInfo_(false),
      deviationAlertPending_(false),
      deviationBlinkMs_(0),
      deviationBlinkToggle_(false),
      printTrackingDebug_(false),
      easterEggCounter_(0),
      activeFault_(FAULT_NONE),
      errorBlinkMs_(0),
      errorChirpMs_(0),
      errorBlinkToggle_(false),
      errorChirpMuted_(false),
      resetHoldInErrorMs_(0),
      homingPhase_(HOMING_PHASE_CLEAR_FWD),
      homingTraveledMm_(0.0f),
      homingClearStartMm_(-1.0f),
      homingSettleStartMs_(0),
      calibrationFinished_(false),
      measuredCalibrationMm_(0.0f),
      calibrationBlinkMs_(0),
      calibrationBlinkToggle_(false),
      resetHoldInCalMs_(0),
      resetHoldTiming_(false),
      resetHandledInState_(false),
      startMs_(0),
      elapsedMs_(0),
      timeoutMs_(0),
      setupCountdownMs_(0) {}

void StateMachine::begin() {
  easterEggCounter_        = 0;
  state_                   = INITIAL_PLATFORM_STATE;
  totalTravelDistanceMm_   = SettingsStorage::loadTravelDistance();
  savedTravelDistanceMm_   = totalTravelDistanceMm_;

  // Diagnostics check on boot: print fault history
  FaultRecord lastFault = SettingsStorage::loadFaultRecord();
  if (lastFault.magic != 0 && lastFault.code != FAULT_NONE) {
    Serial.print(F("Last Fault: "));
    Serial.println(SettingsStorage::getFaultMnemonic(static_cast<FaultCode>(lastFault.code)));
    if (lastFault.acknowledged == 0) {
      displayUI.flashBootFault(SettingsStorage::getFaultMnemonic(static_cast<FaultCode>(lastFault.code)));
      SettingsStorage::acknowledgeFaultRecord();
    }
  }

  resetVariables();

  // Check for saved Park Record (persistent resume)
  if (SettingsStorage::hasValidParkRecord()) {
    // Physical limit switch priority: if Start switch is physically pressed, ignore stale virtual coordinates
    if (inputManager.isStartSwitchRawActive()) {
      Serial.println(F("Home active: park cleared"));
      SettingsStorage::clearParkRecord();
    } else {
      ParkRecord park = SettingsStorage::loadParkRecord();
      remainingDistanceMm_ = min(park.remainingMm, totalTravelDistanceMm_);
      isHomed_             = (remainingDistanceMm_ > 0.0f);
      platformDrive.setTrackingRate(static_cast<TrackingRate>(park.trackingRate), remainingDistanceMm_, totalTravelDistanceMm_);

      Serial.println(F("Park restored"));

      // Immediately clear signature in EEPROM (fail-safe invalidation)
      SettingsStorage::clearParkRecord();
      displayUI.flashResumeNotification();
    }
  }
  inputManager.suppressNextResetRelease();
}

void StateMachine::resetVariables() {
  setupMode_             = RETURN_POLICY_M;
  returnPolicy_          = DEFAULT_RETURN_POLICY;
  remainingDistanceMm_   = 0.0f;
  travelDistanceMm_      = 0.0f;
  remainingDistanceDone_ = false;
  isHomed_               = false;
  cleanRunEligible_      = false;
  returnShowCalInfo_     = false;
  deviationAlertPending_ = false;
  printTrackingDebug_    = false;
  activeFault_           = FAULT_NONE;
  errorChirpMuted_       = false;
  resetHoldInErrorMs_    = 0;
  resetHoldInCalMs_      = 0;
  resetHoldTiming_       = false;
  resetHandledInState_   = false;

  inputManager.reset();
  platformDrive.resetParameters(remainingDistanceMm_, totalTravelDistanceMm_);
  displayUI.reset(state_);

  const unsigned long now = millis();
  startMs_          = now;
  elapsedMs_        = now;
  timeoutMs_        = 0;
  setupCountdownMs_ = now;
}

void StateMachine::update() {
  switch (state_) {
    case SETUP_ST:
      updateSetupState();
      break;
    case RUNNING_ST:
      updateRunningState();
      break;
    case FAST_FORWARD_ST:
      updateFastForwardState();
      break;
    case RETURN_ST:
      updateReturnState();
      break;
    case WAIT_RETURN_ST:
      updateWaitReturnState();
      break;
    case STOP_ST:
      updateStopState();
      break;
    case ERROR_ST:
      updateErrorState();
      break;
    case HOMING_ST:
      updateHomingState();
      break;
    case CALIBRATION_ST:
      updateCalibrationState();
      break;
  }
}

// ============================================================================
// State Transitions
// ============================================================================

void StateMachine::enterSetupState() {
  state_               = SETUP_ST;
  setupMode_           = RETURN_POLICY_M;
  easterEggCounter_    = 0;
  cleanRunEligible_    = false;
  resetHoldInCalMs_    = 0;
  resetHoldTiming_     = false;
  resetHandledInState_ = true;

  platformDrive.disableMotor();
  platformDrive.stopTimer();
  inputManager.reset();
  inputManager.latchResetHeld();
  displayUI.reset(state_);
  displayUI.playDoubleBeep();

  const unsigned long now = millis();
  elapsedMs_        = now;
  setupCountdownMs_ = now;
}

__attribute__((noinline)) void StateMachine::enterStopState() {
  platformDrive.disableMotor();
  platformDrive.stopTimer();
  displayUI.silenceBuzzer();
  state_ = STOP_ST;
  cleanRunEligible_  = false; // User pause disqualifies adaptive calibration for this run
  returnShowCalInfo_ = false;
  inputManager.reset();
  inputManager.suppressNextResetRelease();
  deviationAlertPending_ = (fabs(totalTravelDistanceMm_ - savedTravelDistanceMm_) >= ADAPTIVE_ALERT_DELTA_MM);
  deviationBlinkMs_      = millis();
  deviationBlinkToggle_  = false;
  displayUI.showStatus("rdy");
  Serial.println(F("Motor disabled -> STOP_ST"));
}

__attribute__((noinline)) void StateMachine::enterRunningFromStop(bool performHomingCheck) {
  // If resting on Start switch at start, trigger precision back-off homing sequence first
  if (performHomingCheck && inputManager.isStartSwitchRawActive()) {
    enterHomingSequence();
    return;
  }

  // Cannot drive forward if already pressing the Stop switch
  if (inputManager.isStopSwitchRawActive()) {
    displayUI.playLowRejectBeep();
    return;
  }

  platformDrive.enableMotor();
  inputManager.resetSwitchDebounce();
  displayUI.playShortBeep();
  state_ = RUNNING_ST;

  if (remainingDistanceMm_ > 0.0f) {
    travelDistanceMm_      = 0.0f;
    remainingDistanceDone_ = false;
    platformDrive.fixTargetRpmForLocation(remainingDistanceMm_, totalTravelDistanceMm_);
  } else {
    if (!remainingDistanceDone_) {
      travelDistanceMm_ = 0.0f;
    }
    platformDrive.setTargetRpm(platformDrive.getNominalRpm());
  }
  platformDrive.applyTargetRpmToStepper();
  platformDrive.startTrackingTimer();

  displayUI.resetCountdownBuzzer();
  Serial.println(F("Motor enabled -> RUNNING_ST"));
  displayUI.showStatus("Strt");

  const unsigned long now = millis();
  timeoutMs_ = now;
  elapsedMs_ = now;
  startMs_   = now;
}

__attribute__((noinline)) void StateMachine::enterHomingSequence() {
  platformDrive.enableMotor();
  inputManager.resetSwitchDebounce();
  state_                 = HOMING_ST;
  homingPhase_           = HOMING_PHASE_CLEAR_FWD;
  homingTraveledMm_      = 0.0f;
  homingClearStartMm_    = -1.0f;
  homingSettleStartMs_   = 0;
  remainingDistanceMm_   = 0.0f;
  travelDistanceMm_      = 0.0f;
  remainingDistanceDone_ = false;
  isHomed_               = false;
  cleanRunEligible_      = false;

  displayUI.showStatus("homE");
  displayUI.playShortBeep();

  // Phase 1: Step forward briskly at 160 RPM to clear switch
  platformDrive.applyRpmToStepper(HOMING_CLEAR_RPM);
  platformDrive.startFastTimer();

  elapsedMs_ = millis();
  Serial.println(F("Homing: Clearing Start switch"));
}

void StateMachine::enterFastForwardFromStop() {
  // Cannot fast-forward if already pressing the Stop switch
  if (inputManager.isStopSwitchRawActive()) {
    displayUI.playLowRejectBeep();
    return;
  }

  if (inputManager.isStartSwitchRawActive()) {
    remainingDistanceMm_   = totalTravelDistanceMm_;
    remainingDistanceDone_ = false;
    travelDistanceMm_      = 0.0f;
    isHomed_               = true;
  } else if (remainingDistanceMm_ > 0.0f) {
    travelDistanceMm_      = 0.0f;
    remainingDistanceDone_ = false;
  } else if (!remainingDistanceDone_) {
    travelDistanceMm_ = 0.0f;
  }

  platformDrive.enableMotor();
  inputManager.resetSwitchDebounce();
  state_ = FAST_FORWARD_ST;
  cleanRunEligible_ = false;
  displayUI.resetModeToggleRequest();

  const float ffRpm = -platformDrive.getReturnRpm();
  platformDrive.setTargetRpm(ffRpm);
  platformDrive.applyRpmToStepper(ffRpm);
  platformDrive.startFastTimer();

  const unsigned long now = millis();
  timeoutMs_ = now;
  elapsedMs_ = now;
  startMs_   = now;

  displayUI.showStatus("ff");
  displayUI.playShortBeep();
  Serial.println(F("Fast-Forward started from STOP_ST"));
}

__attribute__((noinline)) void StateMachine::startReturnMotion() {
  remainingDistanceDone_ = false;
  travelDistanceMm_      = 0.0f;
  cleanRunEligible_      = false;
  returnShowCalInfo_     = returnShowCalInfo_ || deviationAlertPending_;

  platformDrive.enableMotor();
  inputManager.resetSwitchDebounce();
  state_ = RETURN_ST;
  displayUI.resetModeToggleRequest();

  const float retRpm = platformDrive.getReturnRpm();
  platformDrive.setTargetRpm(retRpm);
  platformDrive.applyRpmToStepper(retRpm);
  platformDrive.startFastTimer();

  const unsigned long now = millis();
  timeoutMs_ = now;
  elapsedMs_ = now;
  startMs_   = now;

  displayUI.showStatus("rtrn");
  Serial.println(F("Motor reversed -> RETURN_ST"));
}

void StateMachine::enterReturnFromRunning(bool convertRemainingDistance) {
  if (convertRemainingDistance && (remainingDistanceMm_ > 0.0f)) {
    remainingDistanceMm_ = totalTravelDistanceMm_ - remainingDistanceMm_;
  }
  startReturnMotion();
}

void StateMachine::enterReturnFromStop() {
  // Cannot rewind if already resting on the Start switch
  if (inputManager.isStartSwitchRawActive()) {
    remainingDistanceMm_   = totalTravelDistanceMm_;
    remainingDistanceDone_ = false;
    travelDistanceMm_      = 0.0f;
    isHomed_               = true;
    displayUI.playLowRejectBeep();
    return;
  }

  // Convert forward remaining distance (to Stop) into reverse remaining distance (to Start)
  if (inputManager.isStopSwitchRawActive()) {
    remainingDistanceMm_ = totalTravelDistanceMm_;
    isHomed_             = true;
  } else if (remainingDistanceDone_) {
    remainingDistanceMm_ = totalTravelDistanceMm_ + travelDistanceMm_;
    isHomed_             = true;
  } else if (remainingDistanceMm_ > 0.0f) {
    remainingDistanceMm_ = totalTravelDistanceMm_ - remainingDistanceMm_;
    if (remainingDistanceMm_ <= 0.0f) {
      remainingDistanceMm_ = 0.01f;
    }
  }
  returnShowCalInfo_ = false;
  startReturnMotion();
  displayUI.playShortBeep();
}

void StateMachine::enterReturnFromWait() {
  remainingDistanceMm_ = totalTravelDistanceMm_;
  isHomed_             = true;
  startReturnMotion();
  displayUI.playShortBeep();
}

__attribute__((noinline)) void StateMachine::enterWaitReturnState() {
  platformDrive.disableMotor();
  platformDrive.stopTimer();
  displayUI.silenceBuzzer();
  remainingDistanceMm_   = 0.0f;
  remainingDistanceDone_ = true;
  travelDistanceMm_      = 0.0f;
  isHomed_               = true;
  deviationBlinkMs_      = millis();
  deviationBlinkToggle_  = false;
  state_                 = WAIT_RETURN_ST;
  Serial.println(F("Motor disabled -> WAIT_RETURN_ST"));
}

void StateMachine::enterCalibrationState() {
  platformDrive.enableMotor();
  inputManager.resetSwitchDebounce();
  state_                  = CALIBRATION_ST;
  calibrationFinished_    = false;
  measuredCalibrationMm_  = 0.0f;
  travelDistanceMm_       = 0.0f;
  remainingDistanceMm_    = 0.0f;
  remainingDistanceDone_  = false;
  isHomed_                = false;
  resetHoldTiming_        = false;
  resetHoldInCalMs_       = 0;
  resetHandledInState_    = inputManager.isResetButtonRawActive();
  if (resetHandledInState_) {
    inputManager.latchResetHeld();
  }

  displayUI.showStatus("CAL ");
  displayUI.playShortBeep();

  platformDrive.applyRpmToStepper(FAST_CALIBRATION_RPM);
  platformDrive.startFastTimer();

  startMs_                = millis();
  elapsedMs_              = startMs_;
  calibrationBlinkMs_     = startMs_;
  calibrationBlinkToggle_ = false;
  Serial.println(F("Fast calibration started"));
}

void StateMachine::executeParkSequence() {
  if (!isHomed_ || remainingDistanceMm_ <= 0.0f) {
    // Cannot park unhomed
    displayUI.flashUnhomedReject();
    displayUI.showStatus("rdy");
    inputManager.reset();
    inputManager.suppressNextResetRelease();
    return;
  }

  platformDrive.disableMotor();
  platformDrive.stopTimer();

  SettingsStorage::saveParkRecord(remainingDistanceMm_, platformDrive.getTrackingRate());
  displayUI.flashParkConfirmation();

  // Wait for Reset button from the park chord to be continuously released for BUTTON_DEBOUNCE_MS
  unsigned long releaseStartMs = millis();
  while ((millis() - releaseStartMs) < BUTTON_DEBOUNCE_MS) {
    if (inputManager.isResetButtonRawActive()) {
      releaseStartMs = millis();
    }
  }
  inputManager.reset();

  state_ = STOP_ST;
  while (true) {
    // Sits safely parked until power switch is turned off. Pressing reset restarts normal loop.
    if (inputManager.isResetButtonDebouncedActive()) {
      SettingsStorage::clearParkRecord(); // Clear stale record on unpark!
      enterStopState();
      break;
    }
    delay(10);
  }
}

__attribute__((noinline)) void StateMachine::enterErrorState(FaultCode code) {
  platformDrive.disableMotor();
  platformDrive.stopTimer();

  const PlatformState previousState = state_;
  const float currentPosMm          = getDistanceFromDepartureMm();
  state_              = ERROR_ST;
  activeFault_        = code;
  errorBlinkMs_       = millis();
  errorChirpMs_       = millis();
  errorBlinkToggle_   = true;
  errorChirpMuted_    = false;
  resetHoldTiming_    = false;
  resetHoldInErrorMs_ = 0;
  cleanRunEligible_   = false;

  Serial.print(F("FAULT: "));
  Serial.print(SettingsStorage::getFaultMnemonic(code));
  Serial.print(F(" @ "));
  Serial.println(currentPosMm, 1);

  SettingsStorage::saveFaultRecord(code, previousState, currentPosMm);
  displayUI.showStatus("Err ");
  displayUI.playErrorAlertPattern();
}

// ============================================================================
// Per-State Update Handlers
// ============================================================================

void StateMachine::updateStopState() {
  if (SafetyMonitor::checkSwitchShort()) {
    return;
  }

  // Handle deviation alert blinking if pending
  if (deviationAlertPending_) {
    const unsigned long now = millis();
    if ((now - deviationBlinkMs_) >= 1500UL) {
      deviationBlinkMs_ = now;
      deviationBlinkToggle_ = !deviationBlinkToggle_;
      displayUI.showStatus(deviationBlinkToggle_ ? "CAL " : "rdy");
    }
  }

  // Process master button events
  const ButtonEvent event = inputManager.pollStopStateEvents();
  switch (event) {
    case BTN_EVENT_RESET_SHORT_TAP:
      deviationAlertPending_ = false;
      enterRunningFromStop(true);
      break;

    case BTN_EVENT_RESET_HOLD_RATE: {
      deviationAlertPending_ = false;
      const TrackingRate rate = platformDrive.cycleTrackingRate(remainingDistanceMm_, totalTravelDistanceMm_);
      displayUI.playRateTone(rate);
      if (rate == MOON_RATE) {
        displayUI.showStatus("Luna");
      }
#ifdef SUN_RATE_SUPPORT
      else if (rate == SUN_RATE) {
        displayUI.showStatus("Sun");
      }
#endif
      else {
        displayUI.showStatus("Side");
      }
      break;
    }

    case BTN_EVENT_RESET_HOLD_SETUP:
      deviationAlertPending_ = false;
      enterSetupState();
      break;

    case BTN_EVENT_PARK_CHORD:
      deviationAlertPending_ = false;
      executeParkSequence();
      break;

    case BTN_EVENT_SLEW_FWD_HOLD:
      deviationAlertPending_ = false;
      enterFastForwardFromStop();
      break;

    case BTN_EVENT_SLEW_REV_HOLD:
      deviationAlertPending_ = false;
      enterReturnFromStop();
      break;

    default:
      break;
  }
}

void StateMachine::updateHomingState() {
  const unsigned long now = millis();
  const unsigned long stepElapsedMs = now - elapsedMs_;
  elapsedMs_ = now;

  if (SafetyMonitor::checkSwitchShort()) {
    return;
  }

  // Immediate abort on Reset tap
  if (inputManager.isResetButtonDebouncedActive()) {
    enterStopState();
    return;
  }

  switch (homingPhase_) {
    case HOMING_PHASE_CLEAR_FWD: {
      // Step forward at 160 RPM until Start switch goes HIGH
      const float stepMm = PlatformDrive::computeLinearTravelMm(HOMING_CLEAR_RPM, stepElapsedMs);
      homingTraveledMm_ += stepMm;

      // Safety check: capped at 6.0 mm travel to prevent runaway on broken switch
      if (homingTraveledMm_ > MAX_HOMING_CLEAR_MM) {
        enterErrorState(FAULT_START_SWITCH_STUCK);
        return;
      }

      // Check if switch opened (HIGH)
      if (!inputManager.isStartSwitchRawActive()) {
        if (homingClearStartMm_ < 0.0f) {
          homingClearStartMm_ = homingTraveledMm_; // Record point where leaf opened
        }
        // Move an extra 0.8 mm continuously past switch leaf release point for solid clearance
        if (homingTraveledMm_ >= (homingClearStartMm_ + HOMING_OVER_CLEAR_MM)) {
          platformDrive.applyRpmToStepper(0.0f);
          homingPhase_         = HOMING_PHASE_SETTLE;
          homingSettleStartMs_ = now;
          homingClearStartMm_  = homingTraveledMm_; // Store total forward clear distance for reverse safety cap
          homingTraveledMm_    = 0.0f;              // Reset for reverse creep measurement
          Serial.println(F("Homing: Switch cleared"));
        }
      } else {
        // Switch bounced back LOW; require continuous clearance past release point
        homingClearStartMm_ = -1.0f;
      }
      break;
    }

    case HOMING_PHASE_SETTLE: {
      if ((now - homingSettleStartMs_) >= 100UL) {
        // Switch to Phase 3: Reverse at -160 RPM
        homingPhase_      = HOMING_PHASE_CREEP_REV;
        homingTraveledMm_ = 0.0f;
        platformDrive.applyRpmToStepper(HOMING_CREEP_RPM);
        Serial.println(F("Homing: Reversing to switch"));
      }
      break;
    }

    case HOMING_PHASE_CREEP_REV: {
      const float stepMm = PlatformDrive::computeLinearTravelMm(fabs(HOMING_CREEP_RPM), stepElapsedMs);
      homingTraveledMm_ += stepMm;

      // Reverse creep safety cap: cannot exceed actual forward clear distance + 2.0 mm
      if (homingTraveledMm_ > (homingClearStartMm_ + 2.0f)) {
        enterErrorState(FAULT_START_SWITCH_STUCK);
        return;
      }

      // Reversing onto switch gently
      if (inputManager.checkStartSwitchTriggered()) {
        platformDrive.applyRpmToStepper(0.0f);
        platformDrive.stopTimer();

        // Locked onto physical mechanical home!
        remainingDistanceMm_   = totalTravelDistanceMm_;
        travelDistanceMm_      = 0.0f;
        remainingDistanceDone_ = false;
        isHomed_               = true;
        cleanRunEligible_      = (platformDrive.getTrackingRate() == SIDEREAL_RATE) &&
                                 (fabs(platformDrive.getAngularVelocityOffsetMicroRad()) < 0.01f);

        displayUI.playShortBeep();
        Serial.println(F("Homing complete: Zero reference locked. Starting tracking."));

        // Transition straight into forward tracking
        enterRunningFromStop(false);
      }
      break;
    }
  }
}

void StateMachine::updateSetupState() {
  const unsigned long now = millis();

  if (inputManager.isResetButtonDebouncedActive()) {
    // Active Reset press/hold refreshes Setup idle timer so holds never time out mid-gesture
    elapsedMs_        = now;
    setupCountdownMs_ = now;

    // Ignore Reset button if it is still held from entering Setup or after a 2s save action
    if (!resetHandledInState_) {
      if (!resetHoldTiming_) {
        resetHoldTiming_  = true;
        resetHoldInCalMs_ = now;
      } else if ((now - resetHoldInCalMs_) >= 2000UL) {
        resetHandledInState_ = true;
        resetHoldTiming_     = false;
        resetHoldInCalMs_    = 0;
        // In ADAPTIVE_CAL_M: 2s hold saves adapted calibration to EEPROM
        if (setupMode_ == ADAPTIVE_CAL_M) {
          savedTravelDistanceMm_ = totalTravelDistanceMm_;
          deviationAlertPending_ = false;
          returnShowCalInfo_     = false;
          SettingsStorage::saveTravelDistance(totalTravelDistanceMm_);
          displayUI.flashCalibrationSaved();
          displayUI.showStatus("Auto");
          elapsedMs_        = millis();
          setupCountdownMs_ = elapsedMs_;
          return;
        }
      }
    }
  } else {
    if (resetHandledInState_) {
      resetHandledInState_ = false;
      resetHoldTiming_     = false;
      resetHoldInCalMs_    = 0;
      elapsedMs_           = now;
      setupCountdownMs_    = now;
    } else if (resetHoldTiming_) {
      // Released reset button: cycle through setup pages (symmetrically debounced)
      const unsigned long holdDur = now - resetHoldInCalMs_;
      resetHoldTiming_  = false;
      resetHoldInCalMs_ = 0;
      if (holdDur >= TAP_MIN_DURATION_MS && holdDur < 1500UL) {
        elapsedMs_        = now;
        setupCountdownMs_ = now;
        if (setupMode_ == RETURN_POLICY_M) {
          setupMode_ = TRAVEL_DISTANCE_M;
          displayUI.showStatus("dist");
        } else if (setupMode_ == TRAVEL_DISTANCE_M) {
          setupMode_ = FAULT_HISTORY_M;
          FaultRecord rec = SettingsStorage::loadFaultRecord();
          displayUI.showStatus(SettingsStorage::getFaultMnemonic(static_cast<FaultCode>(rec.code)));
        } else if (setupMode_ == FAULT_HISTORY_M) {
          setupMode_ = ADAPTIVE_CAL_M;
          displayUI.showStatus("Auto");
        } else if (setupMode_ == ADAPTIVE_CAL_M) {
          setupMode_ = CALIBRATION_M;
          displayUI.showStatus("FCAL");
        } else {
          setupMode_ = RETURN_POLICY_M;
          displayUI.showStatus("Stup");
        }
      }
    }
  }

  if (setupMode_ == TRAVEL_DISTANCE_M) {
    if (inputManager.pollSpeedButtons()) {
      elapsedMs_        = now;
      setupCountdownMs_ = now;
      if (inputManager.isOnlyOneSpeedButtonPressed()) {
        if (inputManager.isSpeedUpPressed()) {
          totalTravelDistanceMm_ += TRAVEL_DISTANCE_STEP_MM;
        } else {
          totalTravelDistanceMm_ -= TRAVEL_DISTANCE_STEP_MM;
        }
        const float minAllowed = DEFAULT_TRAVEL_DISTANCE_MM * TRAVEL_DISTANCE_MIN_RATIO;
        const float maxAllowed = DEFAULT_TRAVEL_DISTANCE_MM * TRAVEL_DISTANCE_MAX_RATIO;
        totalTravelDistanceMm_   = constrain(totalTravelDistanceMm_, minAllowed, maxAllowed);
        if (remainingDistanceMm_ > totalTravelDistanceMm_) {
          remainingDistanceMm_ = totalTravelDistanceMm_;
        }
        savedTravelDistanceMm_ = totalTravelDistanceMm_;
        deviationAlertPending_ = false;
        returnShowCalInfo_     = false;
        displayUI.showFloat(totalTravelDistanceMm_, 1);
      }
    }
  } else if (setupMode_ == FAULT_HISTORY_M) {
    if (inputManager.pollSpeedButtons()) {
      elapsedMs_        = now;
      setupCountdownMs_ = now;
      if (inputManager.isSpeedDownPressed()) {
        SettingsStorage::clearFaultRecord();
        displayUI.showStatus("nonE");
        displayUI.playShortBeep();
      }
    }
  } else if (setupMode_ == ADAPTIVE_CAL_M) {
    if (inputManager.pollSpeedButtons()) {
      elapsedMs_        = now;
      setupCountdownMs_ = now;
    }
    if ((now - elapsedMs_) > 1000UL && (now - elapsedMs_) < (SETUP_TIMEOUT_MS - SETUP_COUNTDOWN_DELAY_MS)) {
      if ((now - setupCountdownMs_) >= 500UL) {
        setupCountdownMs_ = now;
        displayUI.showFloat(totalTravelDistanceMm_, 1);
      }
    }
  } else if (setupMode_ == CALIBRATION_M) {
    if (inputManager.pollSpeedButtons()) {
      // Fast calibration requires starting from the physical Start switch
      if (!inputManager.isStartSwitchRawActive()) {
        displayUI.flashUnhomedReject();
        displayUI.showStatus("FCAL");
        elapsedMs_        = millis();
        setupCountdownMs_ = elapsedMs_;
        return;
      }
      SettingsStorage::saveTravelDistance(savedTravelDistanceMm_);
      enterCalibrationState();
      return;
    }
  } else {
    // RETURN_POLICY_M
    if (inputManager.pollSpeedButtons()) {
      elapsedMs_        = now;
      setupCountdownMs_ = now;
      if (inputManager.isOnlyOneSpeedButtonPressed()) {
        if (inputManager.isSpeedUpPressed()) {
          if (easterEggCounter_ > 0) {
            easterEggCounter_ = (easterEggCounter_ < 10) ? (easterEggCounter_ + 1) : 100;
            Serial.println(easterEggCounter_);
          }

          switch (returnPolicy_) {
            case AUTO_RT:
              returnPolicy_ = MANUAL_RT;
              Serial.println(F("Hand"));
              displayUI.showStatus("HAnd");
              break;
            case MANUAL_RT:
              returnPolicy_ = MEASURE_RT;
              Serial.println(F("Measure"));
              displayUI.showStatus("test");
              break;
            case MEASURE_RT:
              returnPolicy_ = AUTO_RT;
              Serial.println(F("Auto"));
              displayUI.showStatus("Auto");
              break;
          }
        } else {
          // Speed Down button cycles return RPM
          const float newReturnRpm = platformDrive.cycleReturnRpm();
          displayUI.showInteger(static_cast<int>(newReturnRpm));

          if ((easterEggCounter_ % 10) == 4) {
            easterEggCounter_ += 10;
            Serial.println(easterEggCounter_);
          }
        }
      } else {
        easterEggCounter_ = 1;
      }
    }

    if (easterEggCounter_ == 34) {
      displayUI.playEasterEgg();
      easterEggCounter_ = 100;
      elapsedMs_        = millis();
      setupCountdownMs_ = elapsedMs_;
    }
  }

  const unsigned long currentNow   = millis();
  const unsigned long updatedIdleMs = currentNow - elapsedMs_;
  if (updatedIdleMs > SETUP_TIMEOUT_MS) {
    SettingsStorage::saveTravelDistance(savedTravelDistanceMm_);
    platformDrive.fixTargetRpmForLocation(remainingDistanceMm_, totalTravelDistanceMm_);
    enterStopState();
    Serial.println(F("Ready"));
  } else if (updatedIdleMs >= (SETUP_TIMEOUT_MS - SETUP_COUNTDOWN_DELAY_MS)) {
    if ((currentNow - setupCountdownMs_) >= SETUP_COUNTDOWN_REFRESH_MS) {
      const float remainingSec = static_cast<float>(SETUP_TIMEOUT_MS - updatedIdleMs) / 1000.0f;
      displayUI.showFloat(remainingSec, 1);
      setupCountdownMs_ = currentNow;
    }
  }
}

void StateMachine::updateWaitReturnState() {
  if (SafetyMonitor::checkSwitchShort()) {
    return;
  }

  if (returnShowCalInfo_ || deviationAlertPending_) {
    const unsigned long now = millis();
    if ((now - deviationBlinkMs_) >= 1500UL) {
      deviationBlinkMs_     = now;
      deviationBlinkToggle_ = !deviationBlinkToggle_;
      displayUI.showStatus(deviationBlinkToggle_ ? "CAL " : "push");
    }
  }

  if (inputManager.isResetButtonDebouncedActive()) {
    remainingDistanceMm_   = 0.0f;
    remainingDistanceDone_ = true;
    travelDistanceMm_      = 0.0f;
    isHomed_               = true;
    enterStopState();
    return;
  }

  if (inputManager.pollSpeedButtons()) {
    enterReturnFromWait();
  }
}

void StateMachine::updateRunningState() {
  const float lastRpm = platformDrive.getTargetRpm();

  // 0. Hardware sanity checks (debounced)
  if (SafetyMonitor::checkSwitchShort()) {
    return;
  }
  if (SafetyMonitor::checkUnexpectedStartSwitch(getDistanceFromDepartureMm())) {
    return;
  }

  // 1. Check end-of-travel Stop microswitch
  if (inputManager.checkStopSwitchTriggered()) {
    integrateMotionStep(lastRpm);

    if (returnPolicy_ == MEASURE_RT) {
      // Calculate true measured distance from run before any adaptive RAM adjustment
      finishCalibrationMeasurement(getDistanceFromDepartureMm());
      return;
    }

    // Closed-loop adaptive calibration evaluation on clean full runs
    evaluateRunForAdaptiveCalibration();

    if (returnPolicy_ == AUTO_RT) {
      remainingDistanceMm_   = totalTravelDistanceMm_;
      remainingDistanceDone_ = false;
      travelDistanceMm_      = 0.0f;
      isHomed_               = true;
      enterReturnFromRunning(false);
    } else {
      enterWaitReturnState();
      displayUI.showStatus("push");
    }
    return;
  }

  // 2. Check Reset button to pause tracking cleanly back to STOP_ST
  if (inputManager.isResetButtonDebouncedActive()) {
    integrateMotionStep(lastRpm);
    enterStopState();
    return;
  }

  // 3. Direction buttons fine-tune angular velocity during tracking (+/-)
  if (inputManager.pollSpeedButtons()) {
    cleanRunEligible_ = false; // Speed tweak disqualifies adaptive calibration

    if (inputManager.isOnlyOneSpeedButtonPressed()) {
      if (inputManager.isSpeedUpPressed()) {
        platformDrive.adjustAngularVelocity(+ANGULAR_VELOCITY_STEP, remainingDistanceMm_, totalTravelDistanceMm_);
      } else {
        platformDrive.adjustAngularVelocity(-ANGULAR_VELOCITY_STEP, remainingDistanceMm_, totalTravelDistanceMm_);
      }

      timeoutMs_          = millis();
      printTrackingDebug_ = true;
      displayUI.showFloat(platformDrive.getAngularVelocityOffsetMicroRad(), 0);
    }
  }

  // 4. Periodic distance integration, location RPM compensation, buzzer & display update
  const unsigned long now = millis();
  if ((now - elapsedMs_) >= DISPLAY_UPDATE_DELTA_MS) {
    const unsigned long stepElapsedMs = now - elapsedMs_;
    long deltaSec = static_cast<long>((now - startMs_) / 1000UL);

    if (remainingDistanceMm_ > 0.0f) {
      remainingDistanceMm_ -= PlatformDrive::computeLinearTravelMm(lastRpm, stepElapsedMs);
      if (remainingDistanceMm_ <= 0.0f) {
        travelDistanceMm_      = max(0.0f, -remainingDistanceMm_);
        remainingDistanceMm_   = 0.0f;
        remainingDistanceDone_ = true;
        deltaSec               = 0;
        startMs_               = now;
      } else {
        const float remainingTimeSec =
            PlatformDrive::computeRemainingTimeSec(remainingDistanceMm_, platformDrive.getNominalRpm());
        deltaSec = static_cast<long>(remainingTimeSec);

        platformDrive.fixTargetRpmForLocation(remainingDistanceMm_, totalTravelDistanceMm_);
        if (printTrackingDebug_) {
          printTrackingDebug_ = false;
          Serial.print(F("remainingDistance "));
          Serial.print(remainingDistanceMm_);
          Serial.print(F("   targetRPM: "));
          Serial.println(platformDrive.getTargetRpm(), 4);
        }
        platformDrive.applyTargetRpmToStepper();
        displayUI.updateCountdownBuzzer(remainingTimeSec);
      }

      if (SafetyMonitor::checkHomedDeparture(
              totalTravelDistanceMm_,
              remainingDistanceMm_,
              inputManager.isStartSwitchDebouncedActive(),
              FAULT_START_SWITCH_STUCK)) {
        return;
      }
    } else {
      travelDistanceMm_ += PlatformDrive::computeLinearTravelMm(lastRpm, stepElapsedMs);
      if (SafetyMonitor::checkUnhomedOrOverrun(
              totalTravelDistanceMm_,
              travelDistanceMm_,
              remainingDistanceDone_,
              inputManager.isStartSwitchDebouncedActive(),
              FAULT_START_SWITCH_STUCK,
              FAULT_OVERTRAVEL_FORWARD,
              ERROR_TRAVEL_DISTANCE_MM)) {
        return;
      }
    }

    if ((now - timeoutMs_) >= DISPLAY_TIMEOUT_MS) {
      if (returnPolicy_ == MEASURE_RT) {
        displayUI.showTravelDistance(totalTravelDistanceMm_, remainingDistanceMm_, travelDistanceMm_);
      } else {
        displayUI.showTimeMmSs(deltaSec);
      }
    }

    elapsedMs_ = now;
  }
}

void StateMachine::updateFastForwardState() {
  const float effectiveRpm = FAST_MOVEMENT_FIX_RATIO * platformDrive.getTargetRpm();

  if (SafetyMonitor::checkSwitchShort()) {
    return;
  }
  if (SafetyMonitor::checkUnexpectedStartSwitch(getDistanceFromDepartureMm())) {
    return;
  }

  // Tap Reset at any time halts fast movement back to STOP_ST
  if (inputManager.isResetButtonDebouncedActive()) {
    integrateMotionStep(effectiveRpm);
    enterStopState();
    return;
  }

  if (inputManager.checkStopSwitchTriggered()) {
    enterWaitReturnState();
    displayUI.showStatus("push");
    return;
  }

  updateFastMotionProgress(
      effectiveRpm,
      true,
      inputManager.isStartSwitchDebouncedActive(),
      FAULT_START_SWITCH_STUCK,
      FAULT_OVERTRAVEL_FORWARD,
      END_ERROR_TRAVEL_DISTANCE_MM);
}

void StateMachine::updateReturnState() {
  const float effectiveRpm = -platformDrive.getReturnRpm() * FAST_MOVEMENT_FIX_RATIO;

  if (SafetyMonitor::checkSwitchShort()) {
    return;
  }

  // Tap Reset at any time halts return movement back to STOP_ST
  if (inputManager.isResetButtonDebouncedActive()) {
    integrateMotionStep(effectiveRpm);

    // Convert reverse remaining distance (to Start) back into forward coordinates (to Stop)
    if (remainingDistanceMm_ > 0.0f) {
      const float forwardRemainingMm = totalTravelDistanceMm_ - remainingDistanceMm_;
      if (forwardRemainingMm < 0.0f) {
        travelDistanceMm_      = -forwardRemainingMm;
        remainingDistanceMm_   = 0.0f;
        remainingDistanceDone_ = true;
      } else if (forwardRemainingMm == 0.0f) {
        travelDistanceMm_ = 0.0f;
        if (inputManager.isStopSwitchRawActive()) {
          remainingDistanceMm_   = 0.0f;
          remainingDistanceDone_ = true;
        } else {
          remainingDistanceMm_   = 0.01f;
          remainingDistanceDone_ = false;
        }
      } else {
        remainingDistanceMm_   = forwardRemainingMm;
        remainingDistanceDone_ = false;
        travelDistanceMm_      = 0.0f;
      }
      isHomed_ = true;
    } else if (remainingDistanceDone_) {
      remainingDistanceMm_   = totalTravelDistanceMm_;
      remainingDistanceDone_ = false;
      travelDistanceMm_      = 0.0f;
      isHomed_               = true;
    } else {
      remainingDistanceMm_   = 0.0f;
      remainingDistanceDone_ = false;
      travelDistanceMm_      = 0.0f;
      isHomed_               = false;
    }
    enterStopState();
    return;
  }

  if (SafetyMonitor::checkUnexpectedStopSwitch(getDistanceFromDepartureMm())) {
    return;
  }

  if (inputManager.checkStartSwitchTriggered()) {
    if (platformDrive.getTargetSpeedStepsPerSec() <= 0.0f) {
      remainingDistanceMm_   = totalTravelDistanceMm_;
      travelDistanceMm_      = 0.0f;
      remainingDistanceDone_ = false;
      isHomed_               = true;
      cleanRunEligible_      = (platformDrive.getTrackingRate() == SIDEREAL_RATE) &&
                               (fabs(platformDrive.getAngularVelocityOffsetMicroRad()) < 0.01f);
      enterRunningFromStop(false);
    }
    return;
  }

  updateFastMotionProgress(
      effectiveRpm,
      false,
      inputManager.isStopSwitchDebouncedActive(),
      FAULT_STOP_SWITCH_STUCK,
      FAULT_OVERTRAVEL_RETURN,
      END_ERROR_TRAVEL_DISTANCE_MM);
}

void StateMachine::updateCalibrationState() {
  const unsigned long now = millis();

  if (!calibrationFinished_) {
    travelDistanceMm_ = PlatformDrive::computeLinearTravelMm(
        FAST_CALIBRATION_RPM * FAST_MOVEMENT_FIX_RATIO,
        now - startMs_);
    elapsedMs_ = now;

    // Safety checks during calibration travel
    if (SafetyMonitor::checkSwitchShort()) {
      return;
    }
    if (SafetyMonitor::checkUnexpectedStartSwitch(travelDistanceMm_)) {
      return;
    }
    if ((travelDistanceMm_ > ERROR_TRAVEL_DISTANCE_MM) && inputManager.isStartSwitchDebouncedActive()) {
      enterErrorState(FAULT_START_SWITCH_STUCK);
      return;
    }
    if (travelDistanceMm_ > (DEFAULT_TRAVEL_DISTANCE_MM * TRAVEL_DISTANCE_MAX_RATIO + END_ERROR_TRAVEL_DISTANCE_MM)) {
      enterErrorState(FAULT_OVERTRAVEL_FORWARD);
      return;
    }

    // Tap Reset halts calibration cleanly back to STOP_ST
    if (inputManager.isResetButtonDebouncedActive()) {
      enterStopState();
      return;
    }

    // Display update during fast calibration motion
    if ((now - calibrationBlinkMs_) >= 1500UL) {
      calibrationBlinkMs_ = now;
      displayUI.showFloat(travelDistanceMm_, 1);
    }

    // Stop switch reached!
    if (inputManager.checkStopSwitchTriggered()) {
      finishCalibrationMeasurement(travelDistanceMm_);
    }
  } else {
    // Calibration finished: alternate between "diSt" and measured value
    if ((now - calibrationBlinkMs_) >= 1000UL) {
      calibrationBlinkMs_ = now;
      calibrationBlinkToggle_ = !calibrationBlinkToggle_;
      if (calibrationBlinkToggle_) {
        displayUI.showStatus("diSt");
      } else {
        displayUI.showFloat(measuredCalibrationMm_, 1);
      }
    }

    // Hold Reset for 2s to commit measured distance to EEPROM, or release earlier to exit without saving
    if (inputManager.isResetButtonDebouncedActive()) {
      if (!resetHandledInState_) {
        if (!resetHoldTiming_) {
          resetHoldTiming_  = true;
          resetHoldInCalMs_ = now;
        } else if ((now - resetHoldInCalMs_) >= 2000UL) {
          resetHandledInState_ = true;
          resetHoldTiming_     = false;
          resetHoldInCalMs_    = 0;
          const float minAllowed = DEFAULT_TRAVEL_DISTANCE_MM * TRAVEL_DISTANCE_MIN_RATIO;
          const float maxAllowed = DEFAULT_TRAVEL_DISTANCE_MM * TRAVEL_DISTANCE_MAX_RATIO;
          if (measuredCalibrationMm_ < minAllowed || measuredCalibrationMm_ > maxAllowed) {
            enterStopState();
            displayUI.playLowRejectBeep();
            return;
          }
          totalTravelDistanceMm_ = measuredCalibrationMm_;
          if (remainingDistanceMm_ > totalTravelDistanceMm_) {
            remainingDistanceMm_ = totalTravelDistanceMm_;
          }
          savedTravelDistanceMm_ = totalTravelDistanceMm_;
          deviationAlertPending_ = false;
          returnShowCalInfo_     = false;
          SettingsStorage::saveTravelDistance(totalTravelDistanceMm_);
          displayUI.flashCalibrationSaved();
          enterStopState();
        }
      }
    } else {
      if (resetHandledInState_) {
        resetHandledInState_ = false;
        resetHoldTiming_     = false;
        resetHoldInCalMs_    = 0;
      } else if (resetHoldTiming_) {
        const unsigned long dur = now - resetHoldInCalMs_;
        resetHoldTiming_  = false;
        resetHoldInCalMs_ = 0;
        if (dur >= TAP_MIN_DURATION_MS) {
          enterStopState(); // Clean exit back to ready without saving
        }
      }
    }
  }
}

void StateMachine::evaluateRunForAdaptiveCalibration() {
  returnShowCalInfo_ = false;
  if (!cleanRunEligible_ || platformDrive.getTrackingRate() != SIDEREAL_RATE || returnPolicy_ == MEASURE_RT) {
    return;
  }

  // Unified error formula relative to totalTravelDistanceMm_: positive = over-travel, negative = under-travel
  const float deltaMm = travelDistanceMm_ - remainingDistanceMm_;

  Serial.print(F("Clean run completed! Measuring delta: "));
  Serial.print(deltaMm, 2);
  Serial.println(F(" mm"));

  applyAdaptiveCalibrationDelta(deltaMm);
  cleanRunEligible_ = false; // Evaluated once per run
}

void StateMachine::applyAdaptiveCalibrationDelta(float deltaMm) {
  // Ignore tiny deadband (< 0.1 mm) and reject outliers (> 5.0 mm)
  if (fabs(deltaMm) < ADAPTIVE_MIN_DELTA_MM || fabs(deltaMm) > ADAPTIVE_MAX_DELTA_MM) {
    return;
  }

  // Apply gentle low-pass filter (33% step) directly to active RAM travel distance
  totalTravelDistanceMm_ += (ADAPTIVE_FILTER_GAIN * deltaMm);

  // Clamp within sanity range
  const float minAllowed = DEFAULT_TRAVEL_DISTANCE_MM * TRAVEL_DISTANCE_MIN_RATIO;
  const float maxAllowed = DEFAULT_TRAVEL_DISTANCE_MM * TRAVEL_DISTANCE_MAX_RATIO;
  totalTravelDistanceMm_ = constrain(totalTravelDistanceMm_, minAllowed, maxAllowed);

  Serial.print(F("Adapted travel distance auto-tuned to: "));
  Serial.print(totalTravelDistanceMm_, 2);
  Serial.println(F(" mm"));

  returnShowCalInfo_ = true;
  displayUI.playAdaptiveTuneChime();

  // Check if deviation from EEPROM exceeds alert threshold (0.5 mm)
  if (fabs(totalTravelDistanceMm_ - savedTravelDistanceMm_) >= ADAPTIVE_ALERT_DELTA_MM) {
    deviationAlertPending_ = true;
    deviationBlinkMs_      = millis();
  }
}

void StateMachine::updateFastMotionProgress(
    float effectivePositiveRpm,
    bool invertPercentage,
    bool isDepartureSwitchActive,
    FaultCode departureFault,
    FaultCode overrunFault,
    float overrunMarginMm) {
  const unsigned long now = millis();

  if ((now - timeoutMs_) >= DISPLAY_TIMEOUT_MS) {
    if (inputManager.pollSpeedButtons()) {
      if (state_ == RETURN_ST && returnShowCalInfo_ && ((now - timeoutMs_) < 10000UL)) {
        returnShowCalInfo_ = false;
      } else {
        displayUI.requestModeToggle();
      }
    }
  }

  if ((now - elapsedMs_) >= DISPLAY_UPDATE_DELTA_MS) {
    const unsigned long stepElapsedMs = now - elapsedMs_;
    const float deltaMm = PlatformDrive::computeLinearTravelMm(effectivePositiveRpm, stepElapsedMs);

    float percentage = 0.0f;
    long  deltaSec   = static_cast<long>((now - startMs_) / 1000UL);

    if (remainingDistanceMm_ > 0.0f) {
      remainingDistanceMm_ -= deltaMm;
      if (remainingDistanceMm_ <= 0.0f) {
        travelDistanceMm_      = max(0.0f, -remainingDistanceMm_);
        remainingDistanceMm_   = 0.0f;
        remainingDistanceDone_ = true;
        deltaSec               = 0;
        startMs_               = now;
      } else {
        const float ratio = (remainingDistanceMm_ / totalTravelDistanceMm_) * 100.0f;
        percentage = invertPercentage ? (100.0f - ratio) : ratio;

        const float remainingTimeSec =
            PlatformDrive::computeRemainingTimeSec(remainingDistanceMm_, effectivePositiveRpm);
        deltaSec = static_cast<long>(remainingTimeSec);
      }

      if (SafetyMonitor::checkHomedDeparture(
              totalTravelDistanceMm_,
              remainingDistanceMm_,
              isDepartureSwitchActive,
              departureFault)) {
        return;
      }
    } else {
      travelDistanceMm_ += deltaMm;
      if (SafetyMonitor::checkUnhomedOrOverrun(
              totalTravelDistanceMm_,
              travelDistanceMm_,
              remainingDistanceDone_,
              isDepartureSwitchActive,
              departureFault,
              overrunFault,
              overrunMarginMm)) {
        return;
      }
    }

    const unsigned long runElapsedMs = now - timeoutMs_;
    if (state_ == RETURN_ST && returnShowCalInfo_ && (runElapsedMs < 10000UL)) {
      if (runElapsedMs >= DISPLAY_TIMEOUT_MS) {
        if (((runElapsedMs / 1000UL) & 1UL) == 0UL) {
          displayUI.showStatus("CAL ");
        } else {
          displayUI.showFloat(totalTravelDistanceMm_, 1);
        }
      }
    } else if (runElapsedMs >= DISPLAY_TIMEOUT_MS) {
      returnShowCalInfo_ = false;
      displayUI.showProgressView(percentage, deltaSec, remainingDistanceMm_ > 0.0f);
    }

    elapsedMs_ = now;
  }
}

void StateMachine::updateErrorState() {
  const unsigned long now = millis();

  // 1. Alternate display every 1 second between "Err " and the 4-char mnemonic
  if ((now - errorBlinkMs_) >= ERROR_BLINK_INTERVAL_MS) {
    errorBlinkMs_ = now;
    errorBlinkToggle_ = !errorBlinkToggle_;

    if (errorBlinkToggle_) {
      displayUI.showStatus("Err ");
    } else {
      displayUI.showStatus(SettingsStorage::getFaultMnemonic(activeFault_));
    }
  }

  // 2. Periodic reminder chirp every 30 seconds unless muted
  if (!errorChirpMuted_ && ((now - errorChirpMs_) >= ERROR_CHIRP_INTERVAL_MS)) {
    errorChirpMs_ = now;
    displayUI.playReminderChirp();
  }

  // 3. Any speed button press mutes the audio reminder
  if (inputManager.pollSpeedButtons()) {
    errorChirpMuted_ = true;
    displayUI.silenceBuzzer();
  }

  // 4. Recoverable error exit: hold Reset button for 2 seconds
  if (inputManager.isResetButtonDebouncedActive()) {
    if (!resetHoldTiming_) {
      resetHoldTiming_    = true;
      resetHoldInErrorMs_ = now;
    } else if ((now - resetHoldInErrorMs_) >= 2000UL) {
      resetHoldTiming_    = false;
      resetHoldInErrorMs_ = 0;

      // Guard against repeated EEPROM wear if switches remain physically shorted
      if (inputManager.isStartSwitchRawActive() && inputManager.isStopSwitchRawActive()) {
        displayUI.playLowRejectBeep();
        return;
      }

      // Mark fault as acknowledged in EEPROM
      SettingsStorage::acknowledgeFaultRecord();

      // Clear fault and recover cleanly to STOP_ST ("rdy")
      activeFault_           = FAULT_NONE;
      remainingDistanceMm_   = 0.0f;
      travelDistanceMm_      = 0.0f;
      remainingDistanceDone_ = false;
      isHomed_               = false;

      platformDrive.setTargetRpm(platformDrive.getNominalRpm());
      platformDrive.applyTargetRpmToStepper();

      enterStopState();
      displayUI.playShortBeep();
      Serial.println(F("Recovered -> STOP_ST"));
    }
  } else {
    resetHoldTiming_    = false;
    resetHoldInErrorMs_ = 0;
  }
}

__attribute__((noinline)) void StateMachine::finishCalibrationMeasurement(float measuredMm) {
  platformDrive.disableMotor();
  platformDrive.stopTimer();
  state_                  = CALIBRATION_ST;
  returnShowCalInfo_      = false;
  calibrationFinished_    = true;
  measuredCalibrationMm_  = measuredMm;
  calibrationBlinkMs_     = millis();
  calibrationBlinkToggle_ = true;
  resetHoldTiming_        = false;
  resetHoldInCalMs_       = 0;
  resetHandledInState_    = inputManager.isResetButtonRawActive();
  if (resetHandledInState_) {
    inputManager.latchResetHeld();
  }
  remainingDistanceMm_    = 0.0f;
  remainingDistanceDone_  = false;
  travelDistanceMm_       = 0.0f;
  isHomed_                = false;
  displayUI.showStatus("diSt");
  displayUI.playShortBeep();
  Serial.print(F("Measured: "));
  Serial.print(measuredCalibrationMm_, 2);
  Serial.println(F(" mm"));
}

__attribute__((noinline)) void StateMachine::integrateMotionStep(float effectivePositiveRpm) {
  const unsigned long now = millis();
  const unsigned long stepElapsedMs = now - elapsedMs_;
  if (stepElapsedMs == 0) {
    return;
  }

  const float deltaMm = PlatformDrive::computeLinearTravelMm(effectivePositiveRpm, stepElapsedMs);
  if (remainingDistanceMm_ > 0.0f) {
    remainingDistanceMm_ -= deltaMm;
    if (remainingDistanceMm_ <= 0.0f) {
      travelDistanceMm_      = max(0.0f, -remainingDistanceMm_);
      remainingDistanceMm_   = 0.0f;
      remainingDistanceDone_ = true;
      startMs_               = now;
    }
  } else {
    travelDistanceMm_ += deltaMm;
  }
  elapsedMs_ = now;
}

__attribute__((noinline)) float StateMachine::getDistanceFromDepartureMm() const {
  if (state_ == HOMING_ST) {
    return homingTraveledMm_;
  }
  if (state_ == WAIT_RETURN_ST) {
    return totalTravelDistanceMm_;
  }
  if (remainingDistanceMm_ > 0.0f) {
    return (totalTravelDistanceMm_ > remainingDistanceMm_)
        ? (totalTravelDistanceMm_ - remainingDistanceMm_)
        : 0.0f;
  }
  if (remainingDistanceDone_) {
    return totalTravelDistanceMm_ + travelDistanceMm_;
  }
  return travelDistanceMm_;
}
