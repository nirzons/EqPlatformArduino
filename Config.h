#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Firmware version: 12 (Refactored modular architecture - Part A)
// Uncomment to enable Solar tracking rate in addition to Sidereal and Lunar:
// #define SUN_RATE_SUPPORT

// ============================================================================
// Platform Hardware Profiles
// ============================================================================
#define PLATFORM_NIR      1
#define PLATFORM_GUY      2
#define PLATFORM_RENEWED  3

#define ACTIVE_PLATFORM   PLATFORM_NIR

#if ACTIVE_PLATFORM == PLATFORM_NIR
  constexpr float DEFAULT_TRAVEL_DISTANCE_MM = 170.5f;
#elif ACTIVE_PLATFORM == PLATFORM_GUY
  constexpr float DEFAULT_TRAVEL_DISTANCE_MM = 176.0f;
#elif ACTIVE_PLATFORM == PLATFORM_RENEWED
  constexpr float DEFAULT_TRAVEL_DISTANCE_MM = 155.65f;
#else
  #error "Invalid ACTIVE_PLATFORM selection"
#endif

constexpr float TRAVEL_DISTANCE_MIN_RATIO = 0.9f;
constexpr float TRAVEL_DISTANCE_MAX_RATIO = 1.1f;
constexpr float TRAVEL_DISTANCE_STEP_MM   = 0.1f;

// ============================================================================
// EEPROM Layout
// ============================================================================
constexpr int EP_ADDR_TRAVEL_DISTANCE = 0;

// ============================================================================
// Mechanical & Kinematic Parameters (DO NOT MODIFY - Calibrated for Lat 32)
// ============================================================================
constexpr int   MOTOR_GEAR_TEETH     = 12;     // Teeth on motor pinion gear
constexpr int   ROD_GEAR_TEETH       = 30;     // Teeth on lead-screw gear (T8, 1mm lead)
constexpr float PUSH_ARCH_RADIUS_MM  = 634.8f; // Tangential drive radius from polar axis
constexpr float RADIUS_OFFSET_MM     = 0.05f;
constexpr float CORRECTION_FACTOR    = 0.15f;

constexpr float STEPS_PER_REVOLUTION = 1600.0f;
constexpr float MAX_SPEED_MULTIPLIER = 48.0f;

// Tracking angular velocities in radians per minute
constexpr float SIDEREAL_ANGULAR_VELOCITY = 0.0043752741f;  // 2 * PI / 1436.066667
constexpr float SUN_ANGULAR_VELOCITY      = 0.00436332313f; // 2 * PI / 1440.0
constexpr float MOON_ANGULAR_VELOCITY     = 0.004489f;      // Accounts for 13.2 deg/day lunar shift

constexpr float ANGULAR_VELOCITY_STEP     = 0.000001f;      // Manual fine speed step (rad/min)

// Return & fast-travel parameters
constexpr float DEFAULT_RETURN_RPM        = -320.0f;
constexpr float RETURN_RPM_STEP           = 80.0f;
constexpr float FAST_MOVEMENT_FIX_RATIO   = 0.97f;

// Safety travel error thresholds (mm)
constexpr float ERROR_TRAVEL_DISTANCE_MM     = 2.7f;
constexpr float END_ERROR_TRAVEL_DISTANCE_MM = 3.5f;

// ============================================================================
// Pin Assignments (Arduino Uno / Nano)
// ============================================================================
constexpr uint8_t PIN_BUZZER           = 2;
constexpr uint8_t PIN_RESET            = 3;
constexpr uint8_t PIN_STEPPER_DIR      = 4;
constexpr uint8_t PIN_STEPPER_STEP     = 5;
constexpr uint8_t PIN_BTN_SPEED_DOWN   = 6;
constexpr uint8_t PIN_BTN_SPEED_UP     = 7;
constexpr uint8_t PIN_DISPLAY_CLK      = 8;
constexpr uint8_t PIN_DISPLAY_DIO      = 9;
constexpr uint8_t PIN_STEPPER_ENABLE   = 10; // Active LOW (HIGH = motor disabled)
constexpr uint8_t PIN_SWITCH_STOP      = 11; // Active LOW (INPUT_PULLUP)
constexpr uint8_t PIN_SWITCH_START     = 12; // Active LOW (INPUT_PULLUP)
constexpr uint8_t PIN_DISPLAY_5V       = 13; // Supplies 5V to TM1637 display

constexpr uint8_t STEPPER_DRIVER_INTERFACE = 1; // AccelStepper::DRIVER (Step + Dir)

// ============================================================================
// Timing, Display & Audio Constants
// ============================================================================
constexpr unsigned long TIMER_FAST_PERIOD_US       = 30UL;   // 30 us ISR period for FF / Return
constexpr unsigned long TIMER_SLOW_PERIOD_US       = 150UL;  // 150 us ISR period for Tracking

constexpr int           SWITCH_DEBOUNCE_SAMPLES    = 4;      // Consecutive LOW reads to confirm limit switch
constexpr unsigned long BUTTON_REPEAT_DELAY_MS     = 200UL;  // Speed button repeat interval

constexpr unsigned long DISPLAY_TIMEOUT_MS         = 2000UL; // Transient status message hold time
constexpr unsigned long DISPLAY_UPDATE_DELTA_MS    = 500UL;  // Periodic display & tracking update interval
constexpr unsigned long SETUP_TIMEOUT_MS           = 8000UL; // Idle timeout to exit Setup and save EEPROM
constexpr unsigned long SETUP_COUNTDOWN_DELAY_MS   = 3000UL; // Show numeric countdown during last 3s of Setup
constexpr unsigned long SETUP_COUNTDOWN_REFRESH_MS = 200UL;

constexpr unsigned long RESET_HOLD_DELAY_MS        = 2000UL; // 2s hold: FF/Return chord or Rate toggle
constexpr unsigned long SETUP_ENTRY_HOLD_MS        = 2UL * RESET_HOLD_DELAY_MS; // 4s hold: Enter Setup

constexpr uint8_t       DISPLAY_DEFAULT_BRIGHTNESS = 0;      // Dimmest level (0..7) to preserve night vision
constexpr uint8_t       DISPLAY_COLON_MASK         = 0b01000000;
constexpr int           DISPLAY_BAR_MIN_LEVEL      = 5;

constexpr unsigned long BUZZER_DURATION_MS         = 40UL;
constexpr unsigned int  BUZZER_FREQ_5MIN_HZ        = 400U;
constexpr float         BUZZER_TIME_5MIN_SEC       = 5.0f * 60.0f;
constexpr unsigned int  BUZZER_FREQ_1MIN_HZ        = 600U;
constexpr float         BUZZER_TIME_1MIN_SEC       = 1.0f * 60.0f;
constexpr unsigned int  BUZZER_FREQ_15SEC_HZ       = 800U;
constexpr float         BUZZER_TIME_15SEC_SEC      = 15.0f;

constexpr unsigned long SERIAL_BAUD_RATE           = 9600UL;

// ============================================================================
// Shared Enumerations
// ============================================================================
enum TrackingRate : uint8_t {
  SIDEREAL_RATE = 0,
  MOON_RATE,
#ifdef SUN_RATE_SUPPORT
  SUN_RATE
#endif
};

enum ReturnPolicy : uint8_t {
  AUTO_RT = 0,
  MANUAL_RT,
  MEASURE_RT
};

enum DisplayMode : uint8_t {
  PERCENTAGE_D = 0,
  BAR_D,
  TIME_D,
  MAX_D
};

enum PlatformState : uint8_t {
  SETUP_ST = 0,
  RUNNING_ST,
  FAST_FORWARD_ST,
  RETURN_ST,
  WAIT_RETURN_ST,
  STOP_ST
};

enum SetupMode : uint8_t {
  RETURN_POLICY_M = 0,
  TRAVEL_DISTANCE_M
};

constexpr ReturnPolicy  DEFAULT_RETURN_POLICY  = AUTO_RT;
constexpr PlatformState INITIAL_PLATFORM_STATE = STOP_ST;

#endif // CONFIG_H

