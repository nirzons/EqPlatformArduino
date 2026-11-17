#ifndef PLATFORM_DRIVE_H
#define PLATFORM_DRIVE_H

#include "Config.h"
#include <AccelStepper.h>

class PlatformDrive {
public:
  PlatformDrive();

  // Initializes motor enable pin and attaches the Timer1 step-generation interrupt.
  void begin();

  // Resets tracking rates, nominal/target RPM, return RPM, and stepper max/target speed.
  void resetParameters(float remainingDistanceMm, float totalTravelDistanceMm);

  // Motor driver power control (DM542 enable pin is active LOW).
  void enableMotor();
  void disableMotor();

  // Hardware Timer1 step-polling control.
  void startTrackingTimer();
  void startFastTimer();
  void stopTimer();

  // Updates nominalRPM and nominalRPM_offset from the current angularVelocity_.
  void recalculateNominalRpm();

  // Adjusts targetRPM based on platform carriage position along the tangential drive
  // to maintain constant angular velocity across the tracking arc.
  void fixTargetRpmForLocation(float remainingDistanceMm, float totalTravelDistanceMm);

  // Converts the specified RPM to steps/sec and atomically updates AccelStepper.
  void applyRpmToStepper(float rpm);

  // Applies the current targetRpm_ to AccelStepper.
  void applyTargetRpmToStepper();

  // Cycles Sidereal -> Lunar (-> Solar if enabled) -> Sidereal and updates target RPM.
  TrackingRate cycleTrackingRate(float remainingDistanceMm, float totalTravelDistanceMm);

  // Explicitly sets tracking rate (Sidereal, Moon, Sun) and updates target RPM.
  void setTrackingRate(TrackingRate rate, float remainingDistanceMm, float totalTravelDistanceMm);

  // Fine-tunes angular velocity by +/- ANGULAR_VELOCITY_STEP during tracking.
  void adjustAngularVelocity(float deltaRadPerMin, float remainingDistanceMm, float totalTravelDistanceMm);

  // Cycles return speed in Setup mode between -320, -240, -160, and -80 RPM.
  float cycleReturnRpm();

  // Linear distance & time helpers (preserving exact gearing & lead-screw formulas).
  static float computeLinearTravelMm(float rpm, unsigned long elapsedMs);
  static float computeRemainingTimeSec(float remainingDistanceMm, float effectiveRpm);

  // Getters & Setters
  TrackingRate getTrackingRate() const { return trackingRate_; }
  float getAngularVelocity() const { return angularVelocity_; }
  float getAngularVelocityOffsetMicroRad() const;
  float getNominalRpm() const { return nominalRpm_; }
  float getTargetRpm() const { return targetRpm_; }
  void  setTargetRpm(float rpm) { targetRpm_ = rpm; }
  float getReturnRpm() const { return returnRpm_; }
  float getTargetSpeedStepsPerSec() const { return targetSpeedStepsPerSec_; }

  // Called from Timer1 ISR to emit step pulses when due.
  void runSpeedIsr();

private:
  AccelStepper stepper_;
  float        pushArchRadius_;
  float        angularVelocity_;
  float        nominalRpm_;
  float        nominalRpmOffset_;
  float        targetRpm_;
  float        returnRpm_;
  int          returnRpmStepDir_;
  TrackingRate trackingRate_;
  float        targetSpeedStepsPerSec_;
  float        maxSpeedStepsPerSec_;
};

extern PlatformDrive platformDrive;

#endif // PLATFORM_DRIVE_H

