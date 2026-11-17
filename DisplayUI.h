#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include "Config.h"
#include <TM1637TinyDisplay.h>

class DisplayUI {
public:
  DisplayUI();

  // Powers on and initializes the TM1637 4-digit display and buzzer pin.
  void begin();

  // Resets display mode, brightness, buzzer state, and shows initial status ("Stup" or "rdy").
  void reset(PlatformState currentState);

  // Status & numeric display helpers
  void showStatus(const char* msg);
  void showInteger(int value);
  void showFloat(float value, uint8_t decimalPlaces);
  void showTimeMmSs(long totalSeconds);
  void showTravelDistance(float totalTravelMm, float remainingMm, float traveledMm);

  // Progress display mode management (used during Fast-Forward and Return)
  void resetModeToggleRequest() { displayModeTogglePending_ = false; }
  void requestModeToggle() { displayModeTogglePending_ = true; }
  void showProgressView(float percentage, long deltaSeconds, bool hasRemainingDistance);

  // Easter egg animation ("Program by Nir Zonshine")
  void playEasterEgg();

  // Audio feedback helpers
  void silenceBuzzer();
  void resetCountdownBuzzer();
  void playShortBeep();
  void playDoubleBeep();
  void playLowRejectBeep();
  void playRateTone(TrackingRate rate);
  void playParkSuccessTone();
  void playSaveTone();
  void playAdaptiveTuneChime();
  void playContinuousErrorTone();
  void updateCountdownBuzzer(float remainingTimeSec);

  // Part B error alert feedback
  void playErrorAlertPattern();         // 3 distinct alarm beeps
  void playBootFaultAlarm();            // Multi-tone boot warning for unacknowledged faults
  void playReminderChirp();             // Brief 30s reminder chirp
  void flashBootFault(const char* mnemonic); // Flash Err / Mnemonic with audible alarm on boot

  // Phase 3 UI alerts & confirmations
  void flashParkConfirmation();         // Flashes PArk, plays chime, shows OFF
  void flashResumeNotification();       // Flashes rESt for 1.5s
  void flashCalibrationSaved();         // Flashes SAVd for 1.5s
  void flashUnhomedReject();            // Flashes noHm when park attempted unhomed

private:
  void cycleDisplayMode();

  TM1637TinyDisplay display_;
  DisplayMode       displayMode_;
  bool              displayModeTogglePending_;
  uint8_t           buzzerStage_;
};

extern DisplayUI displayUI;

#endif // DISPLAY_UI_H
