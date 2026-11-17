#include "InputManager.h"

InputManager inputManager;

InputManager::InputManager()
    : lastRepeatMs_(0),
      repeatCooldownActive_(false),
      speedSettlePending_(false),
      speedSettleStartMs_(0),
      speedUpPressed_(false),
      speedDownPressed_(false),
      startSwitchCount_(0),
      stopSwitchCount_(0),
      bothSwitchesCount_(0),
      startDebouncedCount_(0),
      stopDebouncedCount_(0),
      resetPin_{false, false, 0},
      speedUpPin_{false, false, 0},
      speedDownPin_{false, false, 0},
      resetPressStartMs_(0),
      speedUpPressStartMs_(0),
      speedDownPressStartMs_(0),
      parkChordStartMs_(0),
      resetWasPressed_(false),
      setupFiredWhileHeld_(false),
      parkChordTiming_(false),
      parkFiredWhileHeld_(false),
      speedUpHoldTiming_(false),
      speedDownHoldTiming_(false),
      slewFwdFiredWhileHeld_(false),
      slewRevFiredWhileHeld_(false),
      ignoreResetUntilRelease_(false) {}

void InputManager::begin() {
  pinMode(PIN_RESET, INPUT_PULLUP);
  pinMode(PIN_SWITCH_START, INPUT_PULLUP);
  pinMode(PIN_SWITCH_STOP, INPUT_PULLUP);
  pinMode(PIN_BTN_SPEED_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_SPEED_DOWN, INPUT_PULLUP);
  reset();
}

__attribute__((noinline)) void InputManager::reset() {
  lastRepeatMs_            = 0;
  repeatCooldownActive_    = false;
  speedSettlePending_      = false;
  speedSettleStartMs_      = 0;
  speedUpPressed_          = false;
  speedDownPressed_        = false;
  startSwitchCount_        = 0;
  stopSwitchCount_         = 0;
  bothSwitchesCount_       = 0;
  startDebouncedCount_     = 0;
  stopDebouncedCount_      = 0;
  resetPin_                = {false, false, 0};
  speedUpPin_              = {false, false, 0};
  speedDownPin_            = {false, false, 0};

  resetPressStartMs_       = 0;
  speedUpPressStartMs_     = 0;
  speedDownPressStartMs_   = 0;
  parkChordStartMs_        = 0;
  resetWasPressed_         = false;
  setupFiredWhileHeld_     = false;
  parkChordTiming_         = false;
  parkFiredWhileHeld_      = false;
  speedUpHoldTiming_       = false;
  speedDownHoldTiming_     = false;
  slewFwdFiredWhileHeld_   = false;
  slewRevFiredWhileHeld_   = false;
  ignoreResetUntilRelease_ = false;
}

__attribute__((noinline)) bool InputManager::updateDebouncedButton(uint8_t pin, DebouncedPin& state, unsigned long now) {
  const bool rawLow = (digitalRead(pin) == LOW);
  if (rawLow != state.active) {
    if (!state.transitioning) {
      state.transitioning     = true;
      state.transitionStartMs = now;
    } else if ((now - state.transitionStartMs) >= BUTTON_DEBOUNCE_MS) {
      state.active            = rawLow;
      state.transitioning     = false;
      state.transitionStartMs = 0;
    }
  } else {
    state.transitioning     = false;
    state.transitionStartMs = 0;
  }
  return state.active;
}

__attribute__((noinline)) bool InputManager::pollSpeedButtons() {
  const unsigned long now = millis();
  if (repeatCooldownActive_ && (now - lastRepeatMs_ >= BUTTON_REPEAT_DELAY_MS)) {
    repeatCooldownActive_ = false;
  }

  if (!repeatCooldownActive_) {
    const bool upRaw   = (digitalRead(PIN_BTN_SPEED_UP) == LOW);
    const bool downRaw = (digitalRead(PIN_BTN_SPEED_DOWN) == LOW);
    if (upRaw || downRaw) {
      if (!speedSettlePending_) {
        speedSettlePending_ = true;
        speedSettleStartMs_ = now;
      } else if ((now - speedSettleStartMs_) >= 15UL) {
        speedSettlePending_   = false;
        speedUpPressed_       = upRaw;
        speedDownPressed_     = downRaw;
        lastRepeatMs_         = now;
        repeatCooldownActive_ = true;
        return true;
      }
    } else {
      speedSettlePending_ = false;
    }
  }

  return false;
}

ButtonEvent InputManager::pollStopStateEvents() {
  const unsigned long now = millis();
  const bool resetActive  = updateDebouncedButton(PIN_RESET, resetPin_, now);
  const bool upActive     = updateDebouncedButton(PIN_BTN_SPEED_UP, speedUpPin_, now);
  const bool downActive   = updateDebouncedButton(PIN_BTN_SPEED_DOWN, speedDownPin_, now);

  const bool resetHeld = isActivelyHeld(resetPin_);
  const bool upHeld    = isActivelyHeld(speedUpPin_);
  const bool downHeld  = isActivelyHeld(speedDownPin_);

  // If entering STOP_ST while Reset was already held (e.g. from an E-stop or pause tap),
  // suppress release events until the button has been continuously released for BUTTON_DEBOUNCE_MS.
  if (ignoreResetUntilRelease_) {
    if (!resetActive) {
      ignoreResetUntilRelease_ = false;
      resetWasPressed_         = false;
      setupFiredWhileHeld_     = false;
    }
    return BTN_EVENT_NONE;
  }

  // If both speed buttons are pressed simultaneously in STOP_ST, reject chord
  if (upActive && downActive) {
    speedUpHoldTiming_     = false;
    speedDownHoldTiming_   = false;
    speedUpPressStartMs_   = 0;
    speedDownPressStartMs_ = 0;
    slewFwdFiredWhileHeld_ = false;
    slewRevFiredWhileHeld_ = false;
  }

  // 1. Check any chord with Reset (Park chord = Reset + Speed Down held 2s; Reset + Speed Up rejected)
  if ((resetActive || resetWasPressed_) && (upActive || downActive)) {
    resetWasPressed_       = true;
    setupFiredWhileHeld_   = true; // Any chord with Reset suppresses single-button Reset actions
    speedUpHoldTiming_     = false;
    speedDownHoldTiming_   = false;
    speedUpPressStartMs_   = 0;
    speedDownPressStartMs_ = 0;
    slewFwdFiredWhileHeld_ = false;
    slewRevFiredWhileHeld_ = false;
    if (resetHeld && downHeld && !upActive) {
      if (!parkChordTiming_) {
        parkChordTiming_  = true;
        parkChordStartMs_ = now;
      } else if (!parkFiredWhileHeld_ && (now - parkChordStartMs_ >= PARK_CHORD_HOLD_MS)) {
        parkFiredWhileHeld_ = true;
        return BTN_EVENT_PARK_CHORD;
      }
    } else {
      parkChordTiming_    = false;
      parkChordStartMs_   = 0;
      parkFiredWhileHeld_ = false;
    }
    return BTN_EVENT_NONE;
  } else {
    parkChordTiming_    = false;
    parkChordStartMs_   = 0;
    parkFiredWhileHeld_ = false;
  }

  // 2. Reset button timing (symmetrically debounced 25 ms on press and release)
  if (resetActive) {
    if (!resetWasPressed_) {
      resetWasPressed_     = true;
      resetPressStartMs_   = now;
      setupFiredWhileHeld_ = false;
    } else if (resetHeld && !setupFiredWhileHeld_ && (now - resetPressStartMs_ >= SETUP_ENTRY_HOLD_MS)) {
      // Held >= 3.5s -> Fires setup directly while held
      setupFiredWhileHeld_ = true;
      return BTN_EVENT_RESET_HOLD_SETUP;
    }
  } else if (resetWasPressed_) {
    const unsigned long duration = now - resetPressStartMs_;
    resetWasPressed_ = false;

    if (!setupFiredWhileHeld_) {
      if (duration >= TAP_MIN_DURATION_MS && duration < TAP_MAX_DURATION_MS) {
        return BTN_EVENT_RESET_SHORT_TAP;
      } else if (duration >= RATE_TOGGLE_MIN_HOLD_MS && duration <= RATE_TOGGLE_MAX_HOLD_MS) {
        return BTN_EVENT_RESET_HOLD_RATE;
      }
    }
    setupFiredWhileHeld_ = false;
  }

  // If Reset is currently held (or debouncing release), suppress single-button direction slew actions
  if (resetActive || resetWasPressed_) {
    speedUpHoldTiming_     = false;
    speedDownHoldTiming_   = false;
    speedUpPressStartMs_   = 0;
    speedDownPressStartMs_ = 0;
    slewFwdFiredWhileHeld_ = false;
    slewRevFiredWhileHeld_ = false;
    return BTN_EVENT_NONE;
  }

  // 3. Speed Up button hold in STOP_ST (Hold 2s -> Fast-Forward)
  if (upHeld && !downActive) {
    if (!speedUpHoldTiming_) {
      speedUpHoldTiming_     = true;
      speedUpPressStartMs_   = now;
      slewFwdFiredWhileHeld_ = false;
    } else if (!slewFwdFiredWhileHeld_ && (now - speedUpPressStartMs_ >= SLEW_HOLD_DELAY_MS)) {
      slewFwdFiredWhileHeld_ = true;
      return BTN_EVENT_SLEW_FWD_HOLD;
    }
  } else {
    speedUpHoldTiming_     = false;
    speedUpPressStartMs_   = 0;
    slewFwdFiredWhileHeld_ = false;
  }

  // 4. Speed Down button hold in STOP_ST (Hold 2s -> Fast-Return)
  if (downHeld && !upActive) {
    if (!speedDownHoldTiming_) {
      speedDownHoldTiming_   = true;
      speedDownPressStartMs_ = now;
      slewRevFiredWhileHeld_ = false;
    } else if (!slewRevFiredWhileHeld_ && (now - speedDownPressStartMs_ >= SLEW_HOLD_DELAY_MS)) {
      slewRevFiredWhileHeld_ = true;
      return BTN_EVENT_SLEW_REV_HOLD;
    }
  } else {
    speedDownHoldTiming_   = false;
    speedDownPressStartMs_ = 0;
    slewRevFiredWhileHeld_ = false;
  }

  return BTN_EVENT_NONE;
}

__attribute__((noinline)) bool InputManager::checkStartSwitchTriggered() {
  if (digitalRead(PIN_SWITCH_START) == LOW) {
    startSwitchCount_++;
    if (startSwitchCount_ >= SWITCH_DEBOUNCE_SAMPLES) {
      startSwitchCount_ = 0;
      return true;
    }
  } else {
    startSwitchCount_ = 0;
  }
  return false;
}

__attribute__((noinline)) bool InputManager::checkStopSwitchTriggered() {
  if (digitalRead(PIN_SWITCH_STOP) == LOW) {
    stopSwitchCount_++;
    if (stopSwitchCount_ >= SWITCH_DEBOUNCE_SAMPLES) {
      stopSwitchCount_ = 0;
      return true;
    }
  } else {
    stopSwitchCount_ = 0;
  }
  return false;
}

bool InputManager::checkBothSwitchesActiveDebounced() {
  if ((digitalRead(PIN_SWITCH_START) == LOW) && (digitalRead(PIN_SWITCH_STOP) == LOW)) {
    bothSwitchesCount_++;
    if (bothSwitchesCount_ >= SWITCH_DEBOUNCE_SAMPLES) {
      bothSwitchesCount_ = 0;
      return true;
    }
  } else {
    bothSwitchesCount_ = 0;
  }
  return false;
}

__attribute__((noinline)) bool InputManager::isStartSwitchDebouncedActive() {
  if (digitalRead(PIN_SWITCH_START) == LOW) {
    startDebouncedCount_++;
    if (startDebouncedCount_ >= SWITCH_DEBOUNCE_SAMPLES) {
      startDebouncedCount_ = SWITCH_DEBOUNCE_SAMPLES;
      return true;
    }
  } else {
    startDebouncedCount_ = 0;
  }
  return false;
}

__attribute__((noinline)) bool InputManager::isStopSwitchDebouncedActive() {
  if (digitalRead(PIN_SWITCH_STOP) == LOW) {
    stopDebouncedCount_++;
    if (stopDebouncedCount_ >= SWITCH_DEBOUNCE_SAMPLES) {
      stopDebouncedCount_ = SWITCH_DEBOUNCE_SAMPLES;
      return true;
    }
  } else {
    stopDebouncedCount_ = 0;
  }
  return false;
}

__attribute__((noinline)) bool InputManager::isStartSwitchRawActive() const {
  for (int i = 0; i < SWITCH_DEBOUNCE_SAMPLES; i++) {
    if (digitalRead(PIN_SWITCH_START) != LOW) {
      return false;
    }
  }
  return true;
}

__attribute__((noinline)) bool InputManager::isStopSwitchRawActive() const {
  for (int i = 0; i < SWITCH_DEBOUNCE_SAMPLES; i++) {
    if (digitalRead(PIN_SWITCH_STOP) != LOW) {
      return false;
    }
  }
  return true;
}

bool InputManager::isResetButtonRawActive() const {
  return digitalRead(PIN_RESET) == LOW;
}

__attribute__((noinline)) bool InputManager::isResetButtonDebouncedActive() {
  return updateDebouncedButton(PIN_RESET, resetPin_, millis());
}
