#include "DisplayUI.h"

DisplayUI displayUI;

DisplayUI::DisplayUI()
    : display_(PIN_DISPLAY_CLK, PIN_DISPLAY_DIO),
      displayMode_(BAR_D),
      displayModeTogglePending_(false),
      buzzerStage_(0) {}

void DisplayUI::begin() {
  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);

  pinMode(PIN_DISPLAY_5V, OUTPUT);
  digitalWrite(PIN_DISPLAY_5V, HIGH);

  display_.begin();
  display_.setBrightness(DISPLAY_DEFAULT_BRIGHTNESS);
}

__attribute__((noinline)) void DisplayUI::reset(PlatformState currentState) {
  buzzerStage_ = 0;
  noTone(PIN_BUZZER);

  displayMode_              = BAR_D;
  displayModeTogglePending_ = false;

  display_.setBrightness(DISPLAY_DEFAULT_BRIGHTNESS);

  if (currentState == SETUP_ST) {
    display_.showString("Stup");
  } else if (currentState == ERROR_ST) {
    display_.showString("Err ");
  } else if (currentState == HOMING_ST) {
    display_.showString("homE");
  } else {
    display_.showString("rdy");
  }
}

__attribute__((noinline)) void DisplayUI::showStatus(const char* msg) {
  display_.showString(msg);
}

void DisplayUI::showInteger(int value) {
  display_.showNumber(value);
}

__attribute__((noinline)) void DisplayUI::showFloat(float value, uint8_t decimalPlaces) {
  display_.showNumber(value, decimalPlaces);
}

__attribute__((noinline)) void DisplayUI::showTimeMmSs(long totalSeconds) {
  if (totalSeconds < 0L) {
    totalSeconds = 0L;
  } else if (totalSeconds > 5999L) {
    totalSeconds = 5999L;
  }
  const int mins = static_cast<int>(totalSeconds / 60L);
  const int secs = static_cast<int>(totalSeconds % 60L);
  display_.showNumberDec(mins * 100 + secs, DISPLAY_COLON_MASK, true);
}

void DisplayUI::showTravelDistance(float totalTravelMm, float remainingMm, float traveledMm) {
  showFloat(totalTravelMm - remainingMm + traveledMm, 1);
  Serial.print(traveledMm);
  Serial.print(F(" , "));
  Serial.println(remainingMm);
}

void DisplayUI::cycleDisplayMode() {
  uint8_t next = static_cast<uint8_t>(displayMode_) + 1U;
  if (next >= static_cast<uint8_t>(MAX_D)) {
    next = static_cast<uint8_t>(PERCENTAGE_D);
  }
  displayMode_ = static_cast<DisplayMode>(next);
}

__attribute__((noinline)) void DisplayUI::showProgressView(float percentage, long deltaSeconds, bool hasRemainingDistance) {
  if (displayModeTogglePending_) {
    displayModeTogglePending_ = false;
    cycleDisplayMode();
  }

  const int clampedPct = static_cast<int>(constrain(percentage, 0.0f, 100.0f));
  if ((displayMode_ == TIME_D) || !hasRemainingDistance) {
    showTimeMmSs(deltaSeconds);
  } else if (displayMode_ == PERCENTAGE_D) {
    display_.showString("%", 1, 3);
    display_.showNumber(clampedPct, false, 3, 0);
  } else {
    // BAR_D
    display_.showLevel(DISPLAY_BAR_MIN_LEVEL + clampedPct, false);
  }
}

void DisplayUI::playEasterEgg() {
  for (int count = 0; count < 3; count++) {
    for (int x = 0; x <= 100; x += 10) {
      display_.showLevel(x, true);
      delay(20);
    }
    for (int x = 100; x >= 0; x -= 10) {
      display_.showLevel(x, true);
      delay(20);
    }
  }
  delay(1000);
  display_.showString("Program by Nir Zonshine");
}

__attribute__((noinline)) void DisplayUI::silenceBuzzer() {
  noTone(PIN_BUZZER);
}

void DisplayUI::resetCountdownBuzzer() {
  buzzerStage_ = 0;
}

__attribute__((noinline)) void DisplayUI::playShortBeep() {
  tone(PIN_BUZZER, BUZZER_FREQ_5MIN_HZ, BUZZER_DURATION_MS);
}

__attribute__((noinline)) void DisplayUI::playDoubleBeep() {
  tone(PIN_BUZZER, BUZZER_FREQ_1MIN_HZ, 50);
  delay(80);
  tone(PIN_BUZZER, BUZZER_FREQ_1MIN_HZ, 50);
}

__attribute__((noinline)) void DisplayUI::playLowRejectBeep() {
  tone(PIN_BUZZER, 300, 200);
}

void DisplayUI::playRateTone(TrackingRate rate) {
  if (rate == SIDEREAL_RATE) {
    tone(PIN_BUZZER, 800, 50);
  }
#ifdef SUN_RATE_SUPPORT
  else if (rate == SUN_RATE) {
    tone(PIN_BUZZER, 650, 65);
  }
#endif
  else {
    tone(PIN_BUZZER, 500, 80);
  }
}

void DisplayUI::playParkSuccessTone() {
  tone(PIN_BUZZER, 600, 80);
  delay(100);
  tone(PIN_BUZZER, 900, 150);
}

void DisplayUI::playSaveTone() {
  tone(PIN_BUZZER, 700, 80);
  delay(100);
  tone(PIN_BUZZER, 1000, 120);
}

void DisplayUI::playAdaptiveTuneChime() {
  tone(PIN_BUZZER, 600, 70);
  delay(90);
  tone(PIN_BUZZER, 800, 70);
  delay(90);
  tone(PIN_BUZZER, 1000, 120);
}

void DisplayUI::playContinuousErrorTone() {
  tone(PIN_BUZZER, BUZZER_FREQ_5MIN_HZ);
}

void DisplayUI::updateCountdownBuzzer(float remainingTimeSec) {
  if ((buzzerStage_ == 0) && (remainingTimeSec <= BUZZER_TIME_5MIN_SEC + 1.0f)) {
    tone(PIN_BUZZER, BUZZER_FREQ_5MIN_HZ, BUZZER_DURATION_MS);
    buzzerStage_ = 1;
  } else if ((buzzerStage_ == 1) && (remainingTimeSec <= BUZZER_TIME_1MIN_SEC + 1.0f)) {
    tone(PIN_BUZZER, BUZZER_FREQ_1MIN_HZ, BUZZER_DURATION_MS);
    buzzerStage_ = 2;
  } else if ((buzzerStage_ == 2) && (remainingTimeSec <= BUZZER_TIME_15SEC_SEC + 1.0f)) {
    tone(PIN_BUZZER, BUZZER_FREQ_15SEC_HZ, BUZZER_DURATION_MS);
    buzzerStage_ = 3;
  }
}

void DisplayUI::playErrorAlertPattern() {
  for (int i = 0; i < 3; i++) {
    tone(PIN_BUZZER, ERROR_BUZZER_FREQ_HZ, ERROR_BEEP_DURATION_MS);
    delay(150);
  }
}

void DisplayUI::playBootFaultAlarm() {
  tone(PIN_BUZZER, 400, 100);
  delay(120);
  tone(PIN_BUZZER, 600, 100);
  delay(120);
  tone(PIN_BUZZER, 800, 200);
  delay(220);
}

void DisplayUI::playReminderChirp() {
  tone(PIN_BUZZER, ERROR_BUZZER_FREQ_HZ, 40);
}

void DisplayUI::flashBootFault(const char* mnemonic) {
  playBootFaultAlarm();
  for (int i = 0; i < 3; i++) {
    showStatus("Err ");
    delay(500);
    showStatus(mnemonic);
    delay(500);
  }
}

void DisplayUI::flashParkConfirmation() {
  showStatus("PArk");
  playParkSuccessTone();
  delay(1000);
  showStatus("OFF ");
}

void DisplayUI::flashResumeNotification() {
  showStatus("rESt");
  playDoubleBeep();
  delay(1200);
  showStatus("rdy");
}

__attribute__((noinline)) void DisplayUI::flashCalibrationSaved() {
  showStatus("SAVd");
  playSaveTone();
  delay(1200);
}

__attribute__((noinline)) void DisplayUI::flashUnhomedReject() {
  showStatus("noHm");
  playLowRejectBeep();
  delay(1200);
}
