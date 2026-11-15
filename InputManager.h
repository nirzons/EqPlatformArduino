#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

#include "Config.h"

class InputManager {
public:
  InputManager();

  // Configures button and microswitch pins with internal pull-up resistors.
  void begin();

  // Resets debounce counters and button repeat state.
  void reset();

  // Polls Speed Up / Speed Down buttons with repeat throttling (BUTTON_REPEAT_DELAY_MS).
  // Returns true when a button press or repeat interval fires.
  bool pollSpeedButtons();

  bool isSpeedUpPressed() const { return speedUpPressed_; }
  bool isSpeedDownPressed() const { return speedDownPressed_; }
  bool areBothSpeedButtonsPressed() const { return speedUpPressed_ && speedDownPressed_; }
  bool isOnlyOneSpeedButtonPressed() const { return speedUpPressed_ != speedDownPressed_; }
  bool isRepeatCooldownActive() const { return repeatCooldownActive_; }

  // Consecutive-sample debounced limit switch checks (active LOW).
  // Returns true once SWITCH_DEBOUNCE_SAMPLES consecutive LOW reads are observed,
  // and resets the counter immediately when HIGH is read.
  bool checkStartSwitchTriggered();
  bool checkStopSwitchTriggered();

  // Debounced safety checks:
  // Returns true only if both switches read active for SWITCH_DEBOUNCE_SAMPLES consecutive reads.
  bool checkBothSwitchesActiveDebounced();
  // Returns true if Start switch is debounced active (for direction checks during forward motion).
  bool isStartSwitchDebouncedActive();
  // Returns true if Stop switch is debounced active (for direction checks during return motion).
  bool isStopSwitchDebouncedActive();

  // Raw instantaneous microswitch reads (used by safety checks after travel threshold).
  bool isStartSwitchRawActive() const;
  bool isStopSwitchRawActive() const;

  // Raw Reset button read (active LOW).
  bool isResetButtonRawActive() const;

private:
  unsigned long lastRepeatMs_;
  bool          repeatCooldownActive_;
  bool          speedUpPressed_;
  bool          speedDownPressed_;
  int           startSwitchCount_;
  int           stopSwitchCount_;
};

extern InputManager inputManager;

#endif // INPUT_MANAGER_H

