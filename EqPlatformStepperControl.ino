// ============================================================================
// Equatorial Platform Stepper Controller (Latitude 32)
// Author: Nir Zonshine
// Version 12 (Refactored Modular Architecture)
// ============================================================================

#include "Config.h"
#include "DisplayUI.h"
#include "InputManager.h"
#include "PlatformDrive.h"
#include "StateMachine.h"

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  inputManager.begin();
  displayUI.begin();
  platformDrive.begin();
  stateMachine.begin();
}

void loop() {
  stateMachine.update();
}

