#include "PlatformDrive.h"
#include <TimerOne.h>
#include <math.h>

PlatformDrive platformDrive;

namespace {
void timerRunSpeedHook() {
  platformDrive.runSpeedIsr();
}
} // namespace

PlatformDrive::PlatformDrive()
    : stepper_(STEPPER_DRIVER_INTERFACE, PIN_STEPPER_STEP, PIN_STEPPER_DIR),
      pushArchRadius_(PUSH_ARCH_RADIUS_MM),
      angularVelocity_(SIDEREAL_ANGULAR_VELOCITY),
      nominalRpm_(0.0f),
      nominalRpmOffset_(0.0f),
      targetRpm_(0.0f),
      returnRpm_(DEFAULT_RETURN_RPM),
      returnRpmStepDir_(-1),
      trackingRate_(SIDEREAL_RATE),
      targetSpeedStepsPerSec_(0.0f),
      maxSpeedStepsPerSec_(0.0f) {}

void PlatformDrive::begin() {
  pinMode(PIN_STEPPER_ENABLE, OUTPUT);
  disableMotor();

  Timer1.initialize(TIMER_SLOW_PERIOD_US);
  Timer1.attachInterrupt(timerRunSpeedHook);
  Timer1.stop();
}

void PlatformDrive::resetParameters(float remainingDistanceMm, float totalTravelDistanceMm) {
  disableMotor();

  pushArchRadius_  = PUSH_ARCH_RADIUS_MM;
  angularVelocity_ = SIDEREAL_ANGULAR_VELOCITY;
  trackingRate_    = SIDEREAL_RATE;

  recalculateNominalRpm();
  Serial.print(F("nominalRPM: "));
  Serial.println(nominalRpm_);

  fixTargetRpmForLocation(remainingDistanceMm, totalTravelDistanceMm);

  returnRpm_        = DEFAULT_RETURN_RPM;
  returnRpmStepDir_ = -1;

  targetSpeedStepsPerSec_ = (targetRpm_ * STEPS_PER_REVOLUTION) / 60.0f;
  maxSpeedStepsPerSec_    = MAX_SPEED_MULTIPLIER * (targetRpm_ * STEPS_PER_REVOLUTION) / 60.0f;

  const uint8_t oldSREG = SREG;
  cli();
  stepper_.setMaxSpeed(maxSpeedStepsPerSec_);
  stepper_.setSpeed(targetSpeedStepsPerSec_);
  SREG = oldSREG;
}

void PlatformDrive::enableMotor() {
  digitalWrite(PIN_STEPPER_ENABLE, LOW);
}

void PlatformDrive::disableMotor() {
  digitalWrite(PIN_STEPPER_ENABLE, HIGH);
}

void PlatformDrive::startTrackingTimer() {
  Timer1.initialize(TIMER_SLOW_PERIOD_US);
  Timer1.start();
}

void PlatformDrive::startFastTimer() {
  Timer1.initialize(TIMER_FAST_PERIOD_US);
  Timer1.start();
}

void PlatformDrive::stopTimer() {
  Timer1.stop();
}

__attribute__((noinline)) void PlatformDrive::recalculateNominalRpm() {
  const float gearRatio = static_cast<float>(ROD_GEAR_TEETH) / static_cast<float>(MOTOR_GEAR_TEETH);
  nominalRpm_       = angularVelocity_ * pushArchRadius_ * gearRatio;
  nominalRpmOffset_ = angularVelocity_ * RADIUS_OFFSET_MM * gearRatio;
}

__attribute__((noinline)) void PlatformDrive::fixTargetRpmForLocation(float remainingDistanceMm, float totalTravelDistanceMm) {
  float distanceRatio = remainingDistanceMm - (totalTravelDistanceMm / 2.0f);
  if (distanceRatio > 0.0f) {
    distanceRatio = distanceRatio / (pushArchRadius_ - RADIUS_OFFSET_MM); // tan(alpha)
    targetRpm_ = (nominalRpm_ - nominalRpmOffset_) * cos(distanceRatio * CORRECTION_FACTOR);
  } else {
    distanceRatio = distanceRatio / (pushArchRadius_ + RADIUS_OFFSET_MM); // tan(alpha)
    targetRpm_ = (nominalRpm_ + nominalRpmOffset_) * cos(distanceRatio * CORRECTION_FACTOR);
  }
}

__attribute__((noinline)) void PlatformDrive::applyRpmToStepper(float rpm) {
  targetSpeedStepsPerSec_ = (rpm * STEPS_PER_REVOLUTION) / 60.0f;

  const uint8_t oldSREG = SREG;
  cli();
  stepper_.setSpeed(targetSpeedStepsPerSec_);
  SREG = oldSREG;
}

void PlatformDrive::applyTargetRpmToStepper() {
  applyRpmToStepper(targetRpm_);
}

TrackingRate PlatformDrive::cycleTrackingRate(float remainingDistanceMm, float totalTravelDistanceMm) {
  TrackingRate nextRate = SIDEREAL_RATE;
  if (trackingRate_ == SIDEREAL_RATE) {
    nextRate = MOON_RATE;
  }
#ifdef SUN_RATE_SUPPORT
  else if (trackingRate_ == MOON_RATE) {
    nextRate = SUN_RATE;
  }
#endif
  setTrackingRate(nextRate, remainingDistanceMm, totalTravelDistanceMm);
  return trackingRate_;
}

__attribute__((noinline)) void PlatformDrive::setTrackingRate(TrackingRate rate, float remainingDistanceMm, float totalTravelDistanceMm) {
  trackingRate_ = rate;
  if (trackingRate_ == MOON_RATE) {
    angularVelocity_ = MOON_ANGULAR_VELOCITY;
    Serial.println(F("Moon"));
  }
#ifdef SUN_RATE_SUPPORT
  else if (trackingRate_ == SUN_RATE) {
    angularVelocity_ = SUN_ANGULAR_VELOCITY;
    Serial.println(F("Sun"));
  }
#endif
  else {
    angularVelocity_ = SIDEREAL_ANGULAR_VELOCITY;
    trackingRate_    = SIDEREAL_RATE;
    Serial.println(F("Sidereal"));
  }

  recalculateNominalRpm();
  fixTargetRpmForLocation(remainingDistanceMm, totalTravelDistanceMm);
  applyTargetRpmToStepper();
}

void PlatformDrive::adjustAngularVelocity(float deltaRadPerMin, float remainingDistanceMm, float totalTravelDistanceMm) {
  angularVelocity_ = constrain(
      angularVelocity_ + deltaRadPerMin,
      SIDEREAL_ANGULAR_VELOCITY * 0.5f,
      SIDEREAL_ANGULAR_VELOCITY * 1.5f);
  recalculateNominalRpm();
  fixTargetRpmForLocation(remainingDistanceMm, totalTravelDistanceMm);
  applyTargetRpmToStepper();

  Serial.print(F("New angular velocity: "));
  Serial.println(angularVelocity_ * 1000000.0f);
}

float PlatformDrive::cycleReturnRpm() {
  returnRpm_ = returnRpm_ + (RETURN_RPM_STEP * static_cast<float>(returnRpmStepDir_));
  if ((returnRpm_ >= 0.0f) || (returnRpm_ < DEFAULT_RETURN_RPM)) {
    returnRpmStepDir_ = -returnRpmStepDir_;
    returnRpm_ = returnRpm_ + (RETURN_RPM_STEP * 2.0f * static_cast<float>(returnRpmStepDir_));
  }

  Serial.print(F("New return speed: "));
  Serial.println(returnRpm_);
  return returnRpm_;
}

__attribute__((noinline)) float PlatformDrive::computeLinearTravelMm(float rpm, unsigned long elapsedMs) {
  const float invGearRatio = static_cast<float>(MOTOR_GEAR_TEETH) / static_cast<float>(ROD_GEAR_TEETH);
  return rpm * invGearRatio * static_cast<float>(elapsedMs) / 60000.0f;
}

__attribute__((noinline)) float PlatformDrive::computeRemainingTimeSec(float remainingDistanceMm, float effectiveRpm) {
  if (remainingDistanceMm <= 0.0f || effectiveRpm <= 0.0001f) {
    return 0.0f;
  }
  const float invGearRatio = static_cast<float>(MOTOR_GEAR_TEETH) / static_cast<float>(ROD_GEAR_TEETH);
  return (remainingDistanceMm / (effectiveRpm * invGearRatio)) * 60.0f;
}

float PlatformDrive::getAngularVelocityOffsetMicroRad() const {
  float baseVelocity = SIDEREAL_ANGULAR_VELOCITY;
  if (trackingRate_ == MOON_RATE) {
    baseVelocity = MOON_ANGULAR_VELOCITY;
  }
#ifdef SUN_RATE_SUPPORT
  else if (trackingRate_ == SUN_RATE) {
    baseVelocity = SUN_ANGULAR_VELOCITY;
  }
#endif
  return (angularVelocity_ - baseVelocity) * 1000000.0f;
}

void PlatformDrive::runSpeedIsr() {
  stepper_.runSpeed();
}

