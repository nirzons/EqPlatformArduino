#include "InputManager.h"

InputManager inputManager;

InputManager::InputManager()
    : lastRepeatMs_(0),
      repeatCooldownActive_(false),
      speedUpPressed_(false),
      speedDownPressed_(false),
      startSwitchCount_(0),
      stopSwitchCount_(0) {}

void InputManager::begin() {
  pinMode(PIN_RESET, INPUT_PULLUP);
  pinMode(PIN_SWITCH_START, INPUT_PULLUP);
  pinMode(PIN_SWITCH_STOP, INPUT_PULLUP);
  pinMode(PIN_BTN_SPEED_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_SPEED_DOWN, INPUT_PULLUP);
  reset();
}

void InputManager::reset() {
  lastRepeatMs_         = 0;
  repeatCooldownActive_ = false;
  speedUpPressed_       = false;
  speedDownPressed_     = false;
  startSwitchCount_     = 0;
  stopSwitchCount_      = 0;
}

bool InputManager::pollSpeedButtons() {
  const unsigned long now = millis();
  if (repeatCooldownActive_ && (now - lastRepeatMs_ >= BUTTON_REPEAT_DELAY_MS)) {
    repeatCooldownActive_ = false;
  }

  if (!repeatCooldownActive_) {
    speedUpPressed_   = (digitalRead(PIN_BTN_SPEED_UP) == LOW);
    speedDownPressed_ = (digitalRead(PIN_BTN_SPEED_DOWN) == LOW);

    if (speedUpPressed_ || speedDownPressed_) {
      lastRepeatMs_         = now;
      repeatCooldownActive_ = true;
      return true;
    }
  }

  return false;
}

bool InputManager::checkStartSwitchTriggered() {
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

bool InputManager::checkStopSwitchTriggered() {
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

bool InputManager::isStartSwitchRawActive() const {
  return digitalRead(PIN_SWITCH_START) == LOW;
}

bool InputManager::isStopSwitchRawActive() const {
  return digitalRead(PIN_SWITCH_STOP) == LOW;
}

bool InputManager::isResetButtonRawActive() const {
  return digitalRead(PIN_RESET) == LOW;
}

