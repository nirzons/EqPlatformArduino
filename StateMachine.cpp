#include "StateMachine.h"
#include "DisplayUI.h"
#include "InputManager.h"
#include "PlatformDrive.h"
#include "SettingsStorage.h"

StateMachine stateMachine;

namespace {

const MotionFaultCodes RUNNING_FAULT_CODES = {
    "Err1", // Start switch stuck when homed
    "Err2", // Start switch stuck when unhomed
    "Err3", // Full travel overrun without Stop switch
    "Err8", // End-of-remaining-distance overrun without Stop switch
    ERROR_TRAVEL_DISTANCE_MM
};

const MotionFaultCodes FAST_FORWARD_FAULT_CODES = {
    "Err4", // Start switch stuck when homed
    "Err5", // Start switch stuck when unhomed
    "Err6", // Full travel overrun without Stop switch
    "Err7", // End-of-remaining-distance overrun without Stop switch
    END_ERROR_TRAVEL_DISTANCE_MM
};

const MotionFaultCodes RETURN_FAULT_CODES = {
    "Err9", // Stop switch stuck when homed
    "Er11", // Stop switch stuck when unhomed
    "Er12", // Full return overrun without Start switch
    "Err0", // End-of-remaining-distance overrun without Start switch
    END_ERROR_TRAVEL_DISTANCE_MM
};

} // namespace

StateMachine::StateMachine()
    : state_(INITIAL_PLATFORM_STATE),
      setupMode_(RETURN_POLICY_M),
      returnPolicy_(DEFAULT_RETURN_POLICY),
      totalTravelDistanceMm_(DEFAULT_TRAVEL_DISTANCE_MM),
      remainingDistanceMm_(0.0f),
      travelDistanceMm_(0.0f),
      remainingDistanceDone_(false),
      printTrackingDebug_(false),
      resetButtonHandled_(false),
      easterEggCounter_(0),
      startMs_(0),
      elapsedMs_(0),
      timeoutMs_(0),
      setupCountdownMs_(0),
      resetHoldStartMs_(0),
      fastForwardHoldMs_(0),
      fastBackwardHoldMs_(0) {}

void StateMachine::begin() {
  easterEggCounter_      = 0;
  state_                 = INITIAL_PLATFORM_STATE;
  totalTravelDistanceMm_ = SettingsStorage::loadTravelDistance();
  resetVariables();
}

void StateMachine::resetVariables() {
  setupMode_             = RETURN_POLICY_M;
  returnPolicy_          = DEFAULT_RETURN_POLICY;
  remainingDistanceMm_   = 0.0f;
  travelDistanceMm_      = 0.0f;
  remainingDistanceDone_ = false;
  printTrackingDebug_    = false;
  resetButtonHandled_    = false;

  inputManager.reset();
  platformDrive.resetParameters(remainingDistanceMm_, totalTravelDistanceMm_);
  displayUI.reset(state_);

  const unsigned long now = millis();
  startMs_            = now;
  elapsedMs_          = now;
  timeoutMs_          = 0;
  setupCountdownMs_   = now;
  resetHoldStartMs_   = 0;
  fastForwardHoldMs_  = 0;
  fastBackwardHoldMs_ = 0;
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
  }

  // Global 4-second Reset button hold enters Setup from any state
  if (inputManager.isResetButtonRawActive()) {
    const unsigned long now = millis();
    if (resetHoldStartMs_ == 0) {
      resetHoldStartMs_ = now;
    } else if ((now - resetHoldStartMs_) >= SETUP_ENTRY_HOLD_MS) {
      enterSetupState();
    }
  } else {
    resetButtonHandled_ = false;
    resetHoldStartMs_   = 0;
  }
}

// ============================================================================
// State Transitions
// ============================================================================

void StateMachine::enterSetupState() {
  state_ = SETUP_ST;
  platformDrive.stopTimer();
  easterEggCounter_ = 0;
  resetVariables();
  resetButtonHandled_ = true;
}

void StateMachine::enterStopState() {
  platformDrive.disableMotor();
  platformDrive.stopTimer();
  state_ = STOP_ST;
  displayUI.showStatus("rdy");
  Serial.println(F("Motor disabled"));
}

void StateMachine::enterRunningFromStop() {
  platformDrive.enableMotor();
  displayUI.playShortBeep();
  state_ = RUNNING_ST;
  platformDrive.startTrackingTimer();

  displayUI.resetCountdownBuzzer();
  Serial.println(F("Motor enabled"));
  displayUI.showStatus("Strt");

  const unsigned long now = millis();
  timeoutMs_ = now;
  elapsedMs_ = now;
  startMs_   = now;
}

void StateMachine::enterFastForwardFromRunning() {
  state_ = FAST_FORWARD_ST;
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
}

void StateMachine::enterReturnFromRunning(bool convertRemainingDistance) {
  if (convertRemainingDistance && (remainingDistanceMm_ > 0.0f)) {
    remainingDistanceMm_ = totalTravelDistanceMm_ - remainingDistanceMm_;
  }

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
  Serial.println(F("Motor reversed"));
}

void StateMachine::enterReturnFromWait() {
  platformDrive.enableMotor();
  displayUI.playShortBeep();

  if (returnPolicy_ != MEASURE_RT) {
    displayUI.showStatus("rtrn");
  }

  remainingDistanceMm_   = totalTravelDistanceMm_;
  remainingDistanceDone_ = false;
  platformDrive.fixTargetRpmForLocation(remainingDistanceMm_, totalTravelDistanceMm_);

  state_ = RETURN_ST;
  displayUI.resetModeToggleRequest();
  platformDrive.applyRpmToStepper(platformDrive.getReturnRpm());
  platformDrive.startFastTimer();

  const unsigned long now = millis();
  timeoutMs_ = now;
  elapsedMs_ = now;
  startMs_   = now;

  Serial.println(F("Motor reversed"));
}

void StateMachine::enterWaitReturnState(bool restoreVelocitySnapshot) {
  platformDrive.disableMotor();
  platformDrive.stopTimer();

  if (restoreVelocitySnapshot) {
    platformDrive.restoreAngularVelocitySnapshot(remainingDistanceMm_, totalTravelDistanceMm_);
    platformDrive.applyRpmToStepper(platformDrive.getReturnRpm());
  }

  state_ = WAIT_RETURN_ST;
  Serial.println(F("Motor disabled"));
}

// ============================================================================
// Per-State Update Handlers
// ============================================================================

void StateMachine::updateSetupState() {
  displayUI.silenceBuzzer();

  if (inputManager.isResetButtonRawActive() && !resetButtonHandled_) {
    elapsedMs_          = millis();
    resetButtonHandled_ = true;

    if (setupMode_ == RETURN_POLICY_M) {
      setupMode_ = TRAVEL_DISTANCE_M;
      displayUI.showStatus("dist");
    } else {
      setupMode_ = RETURN_POLICY_M;
      displayUI.showStatus("rtrn");
    }
  }

  if (setupMode_ == TRAVEL_DISTANCE_M) {
    if (inputManager.pollSpeedButtons()) {
      elapsedMs_ = millis();
      if (inputManager.isSpeedUpPressed()) {
        totalTravelDistanceMm_ += TRAVEL_DISTANCE_STEP_MM;
      } else {
        totalTravelDistanceMm_ -= TRAVEL_DISTANCE_STEP_MM;
      }
      displayUI.showFloat(totalTravelDistanceMm_, 1);
    }
  } else {
    // RETURN_POLICY_M
    if (inputManager.pollSpeedButtons()) {
      elapsedMs_ = millis();
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
        // Both speed buttons pressed simultaneously
        easterEggCounter_ = 1;
      }
    }

    if (easterEggCounter_ == 34) {
      displayUI.playEasterEgg();
      easterEggCounter_ = 100;
    }
  }

  const unsigned long now    = millis();
  const unsigned long idleMs = now - elapsedMs_;

  if (idleMs > SETUP_TIMEOUT_MS) {
    SettingsStorage::saveTravelDistance(totalTravelDistanceMm_);
    state_ = STOP_ST;
    platformDrive.stopTimer();
    displayUI.showStatus("rdy");
    Serial.println(F("Ready"));
  } else if (idleMs < (SETUP_TIMEOUT_MS - SETUP_COUNTDOWN_DELAY_MS)) {
    setupCountdownMs_ = now;
  } else if ((now - setupCountdownMs_) >= SETUP_COUNTDOWN_REFRESH_MS) {
    const float remainingSec = static_cast<float>(SETUP_TIMEOUT_MS - idleMs) / 1000.0f;
    displayUI.showFloat(remainingSec, 1);
    setupCountdownMs_ = now;
  }
}

void StateMachine::updateStopState() {
  displayUI.silenceBuzzer();

  if (inputManager.pollSpeedButtons()) {
    enterRunningFromStop();
    return;
  }

  if (inputManager.isResetButtonRawActive()) {
    const unsigned long now = millis();
    if (resetHoldStartMs_ == 0) {
      resetHoldStartMs_ = now;
    } else if (!resetButtonHandled_ && ((now - resetHoldStartMs_) >= RESET_HOLD_DELAY_MS)) {
      resetButtonHandled_ = true;
      const TrackingRate rate = platformDrive.cycleTrackingRate(remainingDistanceMm_, totalTravelDistanceMm_);
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
    }
  }
}

void StateMachine::updateWaitReturnState() {
  displayUI.silenceBuzzer();

  if (inputManager.pollSpeedButtons()) {
    enterReturnFromWait();
  }
}

void StateMachine::updateRunningState() {
  const float lastRpm = platformDrive.getTargetRpm();

  // 1. Check end-of-travel Stop microswitch
  if (inputManager.checkStopSwitchTriggered()) {
    if (returnPolicy_ == AUTO_RT) {
      enterReturnFromRunning(false);
    } else {
      enterWaitReturnState(false);
      if (returnPolicy_ == MEASURE_RT) {
        displayUI.showTravelDistance(totalTravelDistanceMm_, remainingDistanceMm_, travelDistanceMm_);
      } else {
        displayUI.showStatus("push");
      }
    }
    remainingDistanceMm_   = totalTravelDistanceMm_;
    remainingDistanceDone_ = false;
    return;
  }

  // 2. Check Reset button to pause/stop tracking
  if (inputManager.isResetButtonRawActive()) {
    enterStopState();
    return;
  }

  // 3. Handle fine speed adjustments and Fast-Forward / Return button chords
  if (inputManager.pollSpeedButtons()) {
    if (inputManager.isOnlyOneSpeedButtonPressed()) {
      if (inputManager.isSpeedUpPressed()) {
        if (fastForwardHoldMs_ == 0) {
          platformDrive.saveAngularVelocitySnapshot();
          fastForwardHoldMs_ = millis();
        }
        fastBackwardHoldMs_ = 0;
        platformDrive.adjustAngularVelocity(+ANGULAR_VELOCITY_STEP, remainingDistanceMm_, totalTravelDistanceMm_);
      } else {
        if (fastBackwardHoldMs_ == 0) {
          platformDrive.saveAngularVelocitySnapshot();
          fastBackwardHoldMs_ = millis();
        }
        fastForwardHoldMs_ = 0;
        platformDrive.adjustAngularVelocity(-ANGULAR_VELOCITY_STEP, remainingDistanceMm_, totalTravelDistanceMm_);
      }

      timeoutMs_          = millis();
      printTrackingDebug_ = true;
      displayUI.showFloat(platformDrive.getAngularVelocityOffsetMicroRad(), 0);
    } else {
      // Both speed buttons pressed simultaneously: check if preceded by 2s single-button hold
      const unsigned long now = millis();
      if ((fastForwardHoldMs_ > 0) && ((now - fastForwardHoldMs_) >= RESET_HOLD_DELAY_MS)) {
        enterFastForwardFromRunning();
        return;
      }
      if ((fastBackwardHoldMs_ > 0) && ((now - fastBackwardHoldMs_) >= RESET_HOLD_DELAY_MS)) {
        platformDrive.restoreAngularVelocitySnapshot(remainingDistanceMm_, totalTravelDistanceMm_);
        enterReturnFromRunning(true);
        return;
      }
    }
  } else if (!inputManager.isRepeatCooldownActive()) {
    fastForwardHoldMs_  = 0;
    fastBackwardHoldMs_ = 0;
  }

  // 4. Periodic distance integration, location RPM compensation, buzzer & display update
  const unsigned long now = millis();
  if ((now - elapsedMs_) >= DISPLAY_UPDATE_DELTA_MS) {
    const unsigned long stepElapsedMs = now - elapsedMs_;
    long deltaSec = static_cast<long>((now - startMs_) / 1000UL);

    if (remainingDistanceMm_ > 0.0f) {
      remainingDistanceMm_ -= PlatformDrive::computeLinearTravelMm(lastRpm, stepElapsedMs);
      if (remainingDistanceMm_ < 0.0f) {
        remainingDistanceMm_   = 0.0f;
        remainingDistanceDone_ = true;
        travelDistanceMm_      = 0.0f;
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

      SafetyMonitor::checkHomedDeparture(
          totalTravelDistanceMm_,
          remainingDistanceMm_,
          inputManager.isStartSwitchRawActive(),
          RUNNING_FAULT_CODES);
    } else {
      travelDistanceMm_ += PlatformDrive::computeLinearTravelMm(lastRpm, stepElapsedMs);
      SafetyMonitor::checkUnhomedOrOverrun(
          totalTravelDistanceMm_,
          travelDistanceMm_,
          remainingDistanceDone_,
          inputManager.isStartSwitchRawActive(),
          RUNNING_FAULT_CODES);
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
  if (inputManager.checkStopSwitchTriggered()) {
    enterWaitReturnState(true);
    displayUI.showStatus("push");
    return;
  }

  if (inputManager.isResetButtonRawActive()) {
    platformDrive.disableMotor();
    platformDrive.restoreAngularVelocitySnapshot(remainingDistanceMm_, totalTravelDistanceMm_);
    platformDrive.applyTargetRpmToStepper();
    state_ = STOP_ST;
    platformDrive.stopTimer();
    displayUI.showStatus("rdy");
    return;
  }

  const float effectiveRpm = FAST_MOVEMENT_FIX_RATIO * platformDrive.getTargetRpm();
  updateFastMotionProgress(
      effectiveRpm,
      true,
      inputManager.isStartSwitchRawActive(),
      FAST_FORWARD_FAULT_CODES);
}

void StateMachine::updateReturnState() {
  displayUI.silenceBuzzer();

  if (inputManager.checkStartSwitchTriggered()) {
    if (platformDrive.getTargetSpeedStepsPerSec() <= 0.0f) {
      if (returnPolicy_ == MEASURE_RT) {
        travelDistanceMm_ = 0.0f;
      }
      remainingDistanceMm_   = totalTravelDistanceMm_;
      remainingDistanceDone_ = false;
      platformDrive.fixTargetRpmForLocation(remainingDistanceMm_, totalTravelDistanceMm_);
      platformDrive.applyTargetRpmToStepper();

      displayUI.showStatus("Strt");
      const unsigned long now = millis();
      timeoutMs_ = now;
      startMs_   = now;
      elapsedMs_ = now;

      state_ = RUNNING_ST;
      platformDrive.startTrackingTimer();
      displayUI.resetCountdownBuzzer();
      Serial.println(F("Motor restarted"));
    }
    return;
  }

  if (inputManager.isResetButtonRawActive()) {
    platformDrive.disableMotor();
    if (remainingDistanceMm_ > 0.0f) {
      remainingDistanceMm_ = totalTravelDistanceMm_ - remainingDistanceMm_;
      platformDrive.fixTargetRpmForLocation(remainingDistanceMm_, totalTravelDistanceMm_);
    } else {
      platformDrive.setTargetRpm(platformDrive.getNominalRpm());
    }
    platformDrive.applyTargetRpmToStepper();
    state_ = STOP_ST;
    platformDrive.stopTimer();
    displayUI.showStatus("rdy");
    return;
  }

  const float effectiveRpm = -platformDrive.getReturnRpm() * FAST_MOVEMENT_FIX_RATIO;
  updateFastMotionProgress(
      effectiveRpm,
      false,
      inputManager.isStopSwitchRawActive(),
      RETURN_FAULT_CODES);
}

void StateMachine::updateFastMotionProgress(
    float effectivePositiveRpm,
    bool invertPercentage,
    bool isDepartureSwitchActive,
    const MotionFaultCodes& faultCodes) {
  const unsigned long now = millis();

  if ((now - timeoutMs_) >= DISPLAY_TIMEOUT_MS) {
    if (inputManager.pollSpeedButtons()) {
      displayUI.requestModeToggle();
    }
  }

  if ((now - elapsedMs_) >= DISPLAY_UPDATE_DELTA_MS) {
    const unsigned long stepElapsedMs = now - elapsedMs_;
    const float deltaMm = PlatformDrive::computeLinearTravelMm(effectivePositiveRpm, stepElapsedMs);

    float percentage = 0.0f;
    long  deltaSec   = static_cast<long>((now - startMs_) / 1000UL);

    if (remainingDistanceMm_ > 0.0f) {
      remainingDistanceMm_ -= deltaMm;
      const float ratio = (remainingDistanceMm_ / totalTravelDistanceMm_) * 100.0f;
      percentage = invertPercentage ? (100.0f - ratio) : ratio;

      const float remainingTimeSec =
          PlatformDrive::computeRemainingTimeSec(remainingDistanceMm_, effectivePositiveRpm);
      deltaSec = static_cast<long>(remainingTimeSec);

      if (remainingDistanceMm_ < 0.0f) {
        remainingDistanceMm_   = 0.0f;
        remainingDistanceDone_ = true;
        travelDistanceMm_      = 0.0f;
        deltaSec               = 0;
        startMs_               = now;
      }

      SafetyMonitor::checkHomedDeparture(
          totalTravelDistanceMm_,
          remainingDistanceMm_,
          isDepartureSwitchActive,
          faultCodes);
    } else {
      travelDistanceMm_ += deltaMm;
      SafetyMonitor::checkUnhomedOrOverrun(
          totalTravelDistanceMm_,
          travelDistanceMm_,
          remainingDistanceDone_,
          isDepartureSwitchActive,
          faultCodes);
    }

    if ((now - timeoutMs_) >= DISPLAY_TIMEOUT_MS) {
      displayUI.showProgressView(percentage, deltaSec, remainingDistanceMm_ > 0.0f);
    }

    elapsedMs_ = now;
  }
}
