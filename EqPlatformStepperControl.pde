#include <AccelStepper.h>
//#include <TM1637TinyDisplayPrivate.h>
#include <TM1637TinyDisplay.h>
#include <TimerOne.h>
#include <EEPROM.h>
// Version 10 - Fix tangent error

void(* resetFunc) (void) = 0;//declare reset function at address 0

//#define SUN_RATE_SUPPORT
// EEPROM MAP
#define EP_ADDR_TRAVEL_DISTANCE 0

// The distance between the start and stop position of the platform in mm
#define TRAVEL_DISTANCE 169.50 // Nir
//#define TRAVEL_DISTANCE 176.0 // Guy
//#define TRAVEL_DISTANCE 155.65 // Renewed
float total_travel_distance;
#define ERROR_TRAVEL_DISTANCE 2.7
#define END_ERROR_TRAVEL_DISTANCE 3.5

// Number of pins on the motor screw
#define MOTOR_PINS 12
// Number of pins on the rod screw
#define ROD_PINS 30

#define PUSH_ARCH_RADIUS 634.8
#define RADIUS_OFFSET 0.05
float push_arch_radius;
//float half_arch_travel_distance = PUSH_ARCH_RADIUS*atan(TRAVEL_DISTANCE/2/PUSH_ARCH_RADIUS);
//float half_arch_travel_distance;
float angular_velocity;
// Tracking rates in radians per minute
#define SIDEREAL_ANGULAR_VELOCITY (0.0043752741) // (2*PI/1436.066667)
#define SUN_ANGULAR_VELOCITY (0.00436332313) // (2*PI/1440)
#define MOON_ANGULAR_VELOCITY (0.004489) // Taking into account 13.2 degrees shift per day
#define SIDEREAL_LINEAR_VELOCITY SIDEREAL_ANGULAR_VELOCITY*PUSH_ARCH_RADIUS
#define SUN_LINEAR_VELOCITY SUN_ANGULAR_VELOCITY*PUSH_ARCH_RADIUS
#define MOON_LINEAR_VELOCITY MOON_ANGULAR_VELOCITY*PUSH_ARCH_RADIUS
#define SIDEREAL_RPM (SIDEREAL_LINEAR_VELOCITY*(float)ROD_PINS/(float)MOTOR_PINS)
#define SUN_RPM (SUN_LINEAR_VELOCITY*(float)ROD_PINS/(float)MOTOR_PINS)
#define MOON_RPM (MOON_LINEAR_VELOCITY*(float)ROD_PINS/(float)MOTOR_PINS)

#define ANGULAR_VELOCITY_STEP  (0.000001)  // For speed control 

#define CORRECTION_FACTOR (0.15)

typedef enum {
  SIDEREAL_RATE = 0,
  MOON_RATE,
#ifdef SUN_RATE_SUPPORT
  SUN_RATE
#endif
} Tracking_rate_e;

// PINS definitions
#define buzzerPin 2
#define resetPin 3
// Define the stepper motor connections pins
#define dirPin 4
#define stepPin 5
// Define pin 13 as GPIO output for display 5V
#define Display5VPin 13
// Define micro switches pins
#define MicroSwitchStartPin 12
#define MicroSwitchStopPin 11
// Define stepper motor enable pin
#define enablePin 10
// defeine the display connections pins
#define CLK 8
#define DIO 9
// Define control buttons pins
#define buttonSpeedUpPin 7
#define buttonSpeedDownPin 6

// Stepper motor interface
#define Interface 1


#define buttonCount 4
#define buttonDelayMs (200)


#define DISPLAY_TIMEOUT (2000) // 2 seconds
//#define DISPLAY_DELTA (1000) // 1 second
#define DISPLAY_DELTA (500) // 1/2 second
#define BUZZER_DURATION (40) // 40ms
#define BUZZER_FREQ1 (400)
#define BUZZER_TIME1 (5*60) // 5 minutes
#define BUZZER_FREQ2 (600)
#define BUZZER_TIME2 (1*60) // 1 minutes
#define BUZZER_FREQ3 (800)
#define BUZZER_TIME3 (15) // 15 seconds
//#define RETURN_RPM (-250.0)
#define RETURN_RPM (-320.0)
//#define RETURN_RPM_STEP (90.0)
#define RETURN_RPM_STEP (80.0)
#define SETUP_TIMEOUT (8000) // 8 seconds
#define SETUP_COUNTDOWN_DELAY (3000) // three seconds
#define RESET_BUTTON_DELAY (2000) // two seconds

#define TIMER_FAST_PERIOD (30)  // runSpeedHook to run every 0.03 miliseconds
#define TIMER_SLOW_PERIOD (150) // runSpeedHook to run every 0.15 miliseconds

typedef enum {
  AUTO_RT = 0,
  MANUAL_RT,
  MEASURE_RT
} Return_policy_e;

typedef enum {
  PERCENTAGE_D,
  BAR_D,
  TIME_D,
  MAX_D
} Display_mode_e;

// Defaut return policity
#define DEFAULT_RETURN AUTO_RT
//#define DEFAULT_RETURN MANUAL_RT

typedef enum {
  SETUP_ST = 0,
  RUNNING_ST,
  FAST_FORWARD_ST,
  RETURN_ST,
  WAIT_RETURN_ST,
  STOP_ST
} Platform_state_e;

// Initial platform state
#define INITIAL_PLATFORM_STATE STOP_ST

typedef enum {
  RETURN_POLICY_M = 0,
  TRAVEL_DISTANCE_M,
} Setup_mode_e;

Setup_mode_e Setup_mode;
Return_policy_e Return_policy;
Platform_state_e Platform_state;

// Create a display object of type TM1637TinyDisplay 4 digit display
TM1637TinyDisplay display(CLK, DIO);

float stepsPerRevolution = 1600.0;
//float stepsPerRevolution = 800.0;

 // Set the desired rotation speed in RPM
//float Original_targetRPM;
float original_angular_velocity;
float nominalRPM, nominalRPM_offset;
float targetRPM;
float returnRPM;
int returnRPM_setup;

int tracking_rate_mode;

// remaining distance till arriving to stop microswitch
float remainingDistance;
float ArchRemainingDistance;
// travel distance when we don't have remaining distance
float travelDistance;
// Set to 1 when finished remainingDistance - for error handling
int remainingDistanceDone;

AccelStepper stepper=AccelStepper(Interface, stepPin, dirPin); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

// Calculate the desired speed based on RPM
float targetSpeed;
float MaxSpeed;

unsigned long buttonPressedTime_ms;
int buttonStartCount;
int buttonStopCount;
int buttonSpeedPushed;

unsigned int DisplayMode, DisplayModeButton;

int buzzerOn;
int reset_button_pushed;

int EECounter;

unsigned long start_ms, finished_ms, elapsed_ms, timeout_ms, setup_countdown_ms, reset_ms, fast_forward_ms, fast_backward_ms;

void init_vars()
{
  digitalWrite (enablePin, HIGH); // Disable motor

  buzzerOn = 0;
  noTone(buzzerPin); // Just to be sure

  DisplayMode = BAR_D;
  DisplayModeButton = 0;
  reset_button_pushed = 0;
  
  // Set the desired rotation speed in RPM
  push_arch_radius = PUSH_ARCH_RADIUS;
  //half_arch_travel_distance = push_arch_radius*atan(TRAVEL_DISTANCE/2/push_arch_radius);
  angular_velocity = SIDEREAL_ANGULAR_VELOCITY;
//  targetRPM = SIDEREAL_RPM;
  tracking_rate_mode = SIDEREAL_RATE;
  nominalRPM = (angular_velocity*push_arch_radius*(float)ROD_PINS/(float)MOTOR_PINS);
  nominalRPM_offset = (angular_velocity*RADIUS_OFFSET*(float)ROD_PINS/(float)MOTOR_PINS);
  Serial.print("nominalRPM: ");
  Serial.println(nominalRPM);
  FixTargetRPMAccordingToLoacation();
  //Original_targetRPM = targetRPM;
  original_angular_velocity = angular_velocity;
  returnRPM = RETURN_RPM;
  returnRPM_setup = -1;
// Calculate the desired speed based on RPM
  targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;
  MaxSpeed = 48*(targetRPM * stepsPerRevolution) / 60.0;

  stepper.setMaxSpeed(MaxSpeed);
  stepper.setSpeed(targetSpeed);	
//  stepper.setAcceleration(MaxSpeed/2);

	// Set the brightness to 5 (0=dimmest 7=brightest)
	display.setBrightness(0);
  Setup_mode = RETURN_POLICY_M;
  Return_policy = DEFAULT_RETURN;

  remainingDistance = 0;
  travelDistance = 0;
  remainingDistanceDone = 0;

	// Write on display according to state
  if (Platform_state == SETUP_ST)
  {
	  display.showString("Stup");
  }
  else
  { // Assuming STOP_ST
	  display.showString("rdy");
  }

  // Starting time
  start_ms = millis();
  elapsed_ms = start_ms;
  timeout_ms = 0;
  reset_ms = 0;
  fast_forward_ms = 0;
  fast_backward_ms = 0;
}

void ReadEEPROMVariables(void)
{
  Serial.println ("Read from EEPROM");
  EEPROM.get(EP_ADDR_TRAVEL_DISTANCE, total_travel_distance);
  Serial.print ("total_travel_distance: ");
  Serial.println(total_travel_distance, 1);    //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
  if (isnan(total_travel_distance) || (total_travel_distance < TRAVEL_DISTANCE*0.9) || (total_travel_distance > TRAVEL_DISTANCE*1.1))
  {
    Serial.print ("total_travel_distance set to default: ");
    total_travel_distance = TRAVEL_DISTANCE;
    Serial.println(total_travel_distance, 1);
  }
  for (int n=0; n<10; n++)
  {
    byte b;
    EEPROM.get(n, b);
    Serial.print (n);
    Serial.print (": ");
    Serial.println(b);
  }
}

void WriteEEPROMVariables(void)
{
  float read_float;
  Serial.println ("Write to EEPROM");
  EEPROM.get(EP_ADDR_TRAVEL_DISTANCE, read_float);
  if (read_float != total_travel_distance)
  {
    Serial.print ("set total_travel_distance to: ");
    Serial.println(total_travel_distance, 1);
    EEPROM.put(EP_ADDR_TRAVEL_DISTANCE, total_travel_distance);
  }
}


////////////////////////////////////////////////////////////////////////////////
void setup()
{  
  EECounter = 0;
    // initialize the serial port:
  Serial.begin(9600);
  // Define buzzer pin as output
  pinMode(buzzerPin, OUTPUT);

  // Define reset pin as input and activate the internal pull-up resistor
  pinMode(resetPin, INPUT_PULLUP);

  // Define micro switch buttons as input and activate the internal pull-up resistor
  pinMode(MicroSwitchStartPin, INPUT_PULLUP);
  pinMode(MicroSwitchStopPin, INPUT_PULLUP);

  pinMode(buttonSpeedUpPin, INPUT_PULLUP);
  pinMode(buttonSpeedDownPin, INPUT_PULLUP);

  pinMode(Display5VPin, OUTPUT);
  digitalWrite (Display5VPin, HIGH); // Give 5V to display
  display.begin();

  pinMode(enablePin, OUTPUT); // Motor enable pin

  Platform_state = INITIAL_PLATFORM_STATE;

  Timer1.initialize(TIMER_SLOW_PERIOD);
  Timer1.attachInterrupt(runSpeedHook); 
  //Timer1.start(); // Done when needed

  ReadEEPROMVariables();

  init_vars();
}

  int MicroSwitchStartValue;
  int MicroSwitchStopValue;
  int buttonSpeedUpValue;
  int buttonSpeedDownValue;


////////////////////////////////////////////////////////////////////////////////
void Set_Error (char* Msg)
{ // Error - Stop platform now
  digitalWrite (enablePin, HIGH); // Disable motor
  Platform_state = STOP_ST;
  Timer1.stop();
  display.showString(Msg);
  tone(buzzerPin,BUZZER_FREQ1);
  while (1);
}

////////////////////////////////////////////////////////////////////////////////
void PrintTravelDistance (void)
{ // Print travel distance with 2 digits after the decimal point
  float localTravelDistance = travelDistance;
  if (localTravelDistance > 100.0)
  {
    localTravelDistance = localTravelDistance - 100;
  }
  display.showNumber(localTravelDistance,2);
}

////////////////////////////////////////////////////////////////////////////////
// This hook is called from within the private version TM1637TinyDisplay
// so that the motor does not stop while writing to the display
void runSpeedHook(void)
{
// if ((Platform_state == RUNNING_ST) || (Platform_state == RETURN_ST) || (Platform_state == FAST_FORWARD_ST))
    stepper.runSpeed();
}

// Updates buttonSpeedUpValue and buttonSpeedDownValue - return LOW is a button is pressed
int read_speed_buttons(void)
{
  if (buttonPressedTime_ms > 0)
  { // Button delay in progress
    if (millis() > buttonPressedTime_ms)
    buttonPressedTime_ms = 0;
  }

  if (buttonPressedTime_ms == 0)
  { // Handle buttons
    buttonSpeedUpValue = digitalRead(buttonSpeedUpPin);
    buttonSpeedDownValue = digitalRead(buttonSpeedDownPin);

    if ((buttonSpeedUpValue == LOW) || (buttonSpeedDownValue == LOW))
    { // Button is pressed
      buttonPressedTime_ms = millis()+buttonDelayMs; // issue new delay
      return LOW;
    }
  }
  return HIGH; 
}

////////////////////////////////////////////////////////////////////////////////

void do_setup_loop(void)
{ // Set up return policity
  noTone(buzzerPin); // Just to be sure

  if (digitalRead(resetPin) == LOW)
  {
    { // Change setup mode
      if (reset_button_pushed == 0)
      {
        elapsed_ms = millis(); // Restart setupt timeout
        reset_button_pushed = 1;
        switch (Setup_mode)
        {
        case RETURN_POLICY_M:
          Setup_mode = TRAVEL_DISTANCE_M;
          display.showString("dist");
          break;
        case TRAVEL_DISTANCE_M:
          Setup_mode = RETURN_POLICY_M;
          display.showString("rtrn");
          break;
        }
      }
    }
  }

  switch (Setup_mode)
  {
  case TRAVEL_DISTANCE_M:
    if (read_speed_buttons() == LOW)
    { // Button is pressed
      elapsed_ms = millis(); // Restart setupt timeout
      if (buttonSpeedUpValue == LOW)
      { // Speed up button - increase travel distance;
        total_travel_distance += 0.1;
      }
      else
      { // Speed down button - decrease travel distance;
        total_travel_distance -= 0.1;
      }
      display.showNumber (total_travel_distance,1);
    }
    break;
  case RETURN_POLICY_M:
    if (read_speed_buttons() == LOW)
    { // Button is pressed
      elapsed_ms = millis(); // Restart setupt timeout
      if (!((buttonSpeedDownValue == LOW) && (buttonSpeedUpValue == LOW)))
      { // Only one button is pressed (disregard when both are pressed)
        if (buttonSpeedUpValue == LOW)
        { // Speed up button - change policy
          if (EECounter > 0)
          {
            if (EECounter < 10)
            {
              EECounter += 1;
            }
            else
            {
              EECounter = 100;
            }
            Serial.println(EECounter);
          }
          switch (Return_policy)
          {
          case AUTO_RT:
            Return_policy = MANUAL_RT;
            Serial.println("Hand");
            display.showString("HAnd");
            break;
          case MANUAL_RT:
            Return_policy = MEASURE_RT;
            Serial.println("Measure");
            display.showString("test");
            break;
          case MEASURE_RT:
            Return_policy = AUTO_RT;
            Serial.println("Auto");
            display.showString("Auto");
            break;
          }
        }
        else
        { 
          { // Speed down button - change return speed
            returnRPM = returnRPM+(RETURN_RPM_STEP)*returnRPM_setup;
            if ((returnRPM >= 0.0) or (returnRPM < RETURN_RPM))
            {
              returnRPM_setup = -returnRPM_setup;
              returnRPM = returnRPM+(RETURN_RPM_STEP)*2*returnRPM_setup;
            }
            display.showNumber((int)returnRPM);
            Serial.print("New return speed: ");
            Serial.println(returnRPM);
          }
          if ((EECounter%10) == 4)
          {
            EECounter += 10;
            Serial.println(EECounter);
          }
        }
      }
      else
      { // Both are pressed
        EECounter = 1;
      }
    }

    if (EECounter == 34)
    {
      for (int count = 0; count < 3; count++)
      {
        for (int x = 0; x <= 100; x = x + 10)
        {
          display.showLevel(x, true);
          delay(20);
        }
        for (int x = 100; x >= 0; x = x - 10)
        {
          display.showLevel(x, true);
          delay(20);
        }
      }
      delay(1000);
      display.showString("Program by Nir Zonshine");
      EECounter = 100;
    }
    break;
  }

  finished_ms = millis();
  // Test setup timeout
  if ((finished_ms-elapsed_ms) > SETUP_TIMEOUT)
  { // Setup timeout expired - finish setup
    WriteEEPROMVariables();
    Platform_state = STOP_ST;
    Timer1.stop();
    display.showString("rdy");
    Serial.println("Ready");
  }
  else
  {
    if ((finished_ms-elapsed_ms) < (SETUP_TIMEOUT-SETUP_COUNTDOWN_DELAY))
    {
        setup_countdown_ms = finished_ms;
    }
    else
    {
      if ((finished_ms - setup_countdown_ms) >= 200)
      {
        display.showNumber((float)(SETUP_TIMEOUT-(finished_ms-elapsed_ms))/1000, 1);
      //  Serial.println(finished_ms-elapsed_ms);
      //  Serial.println(SETUP_TIMEOUT-SETUP_COUNTDOWN_DELAY);
        setup_countdown_ms = finished_ms;
      }
    }
  }
}

void FixTargetRPMAccordingToLoacation(void)
{ // Fix the speed and target RPM according to location
  float distanceRatio = (remainingDistance-(total_travel_distance/2));
  //float alpha = atan(distanceRatio);
  //targetRPM = nominalRPM*cos(alpha*ANGLE_CORRECTION_FACTOR);
  if (distanceRatio>0)
  {
    distanceRatio = distanceRatio/(push_arch_radius-RADIUS_OFFSET); // tan(alpha)
    targetRPM = (nominalRPM-nominalRPM_offset)*cos(distanceRatio*CORRECTION_FACTOR);
  }
  else
  {
    distanceRatio = distanceRatio/(push_arch_radius+RADIUS_OFFSET); // tan(alpha)
    targetRPM = (nominalRPM+nominalRPM_offset)*cos(distanceRatio*CORRECTION_FACTOR);
  }
#if 0
  Serial.print ("nominalRPM ");
  Serial.print(nominalRPM,4);
  Serial.print (" remainingDistance ");
  Serial.print(remainingDistance);
  Serial.print("   targetRPM: ");
  Serial.println(targetRPM,4);
#endif
}

////////////////////////////////////////////////////////////////////////////////
void do_fast_forward_loop(void)
{
  //stepper.runSpeed(); // Activate motor
  float lastRPM = targetRPM;

  // First - test for stop microswitch
  MicroSwitchStopValue = digitalRead(MicroSwitchStopPin);
  
  if (MicroSwitchStopValue == LOW)
  { // Stop Microswitch pushed
    buttonStopCount++;
    if(buttonStopCount == buttonCount)
    {  // Microswitch stop was met - need to reset platform
      { // work like in manual mode (Return_policy == MANUAL_RT)
        digitalWrite (enablePin, HIGH); // Disable motor
        angular_velocity = original_angular_velocity;

        nominalRPM = (angular_velocity*push_arch_radius*(float)ROD_PINS/(float)MOTOR_PINS);
        nominalRPM_offset = (angular_velocity*RADIUS_OFFSET*(float)ROD_PINS/(float)MOTOR_PINS);
        FixTargetRPMAccordingToLoacation();
        targetSpeed = (returnRPM * stepsPerRevolution) / 60.0;
        stepper.setSpeed(targetSpeed);
        Platform_state = WAIT_RETURN_ST;
        Timer1.stop();
        display.showString("push");
      }
      buttonStopCount = 0;
    }
  }
  else
  { // Button released
    buttonStopCount = 0;
  }

  if (digitalRead(resetPin) == LOW)
  {
      digitalWrite (enablePin, HIGH); // Disable motor
      angular_velocity = original_angular_velocity;
      nominalRPM = (angular_velocity*push_arch_radius*(float)ROD_PINS/(float)MOTOR_PINS);
      nominalRPM_offset = (angular_velocity*RADIUS_OFFSET*(float)ROD_PINS/(float)MOTOR_PINS);
      FixTargetRPMAccordingToLoacation();
      targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;
      stepper.setSpeed(targetSpeed);
      Platform_state = STOP_ST;
      Timer1.stop();
      display.showString("rdy");
  }

  finished_ms = millis();

  if ((finished_ms - timeout_ms) >= DISPLAY_TIMEOUT)
  { // Disregard keys pressed before timeout
    if (read_speed_buttons() == LOW)
    {
      DisplayModeButton = 1;
    }
  }

  if (finished_ms - elapsed_ms >= DISPLAY_DELTA)
  { // One second has passed. Update display
    float percentage = 0;
    long delta_sec = (finished_ms-start_ms)/1000;
    if (remainingDistance > 0)
    { // Print remaining time
      remainingDistance = remainingDistance - (lastRPM * (float)MOTOR_PINS/ROD_PINS * (float)(finished_ms - elapsed_ms)/60000);
      percentage = 100.0-(remainingDistance / total_travel_distance) * 100.0;
      float remainingTime = remainingDistance / (targetRPM * (float)MOTOR_PINS/ROD_PINS)*60;
      delta_sec = remainingTime;
      if (remainingDistance<0)
      {
        remainingDistance = 0;
        remainingDistanceDone = 1;
        travelDistance = 0;
        delta_sec = 0;
        start_ms = finished_ms;
      }

      // Test for error condition - Start microswitch should be off after some time
      if (total_travel_distance - remainingDistance > ERROR_TRAVEL_DISTANCE)
      { // After ERROR_TRAVEL_DISTANCE start microswitch must be off
        MicroSwitchStartValue = digitalRead(MicroSwitchStartPin);
  
        if (MicroSwitchStartValue == LOW)
        { // Start Microswitch pushed - Error - Stop platform now
          Set_Error("Err4");
        }
      }
    }
    else
    {  // Calculate travel distance
      travelDistance = travelDistance + (lastRPM * (float)MOTOR_PINS/ROD_PINS * (float)(finished_ms - elapsed_ms)/60000);
      if (travelDistance > ERROR_TRAVEL_DISTANCE)
      { // After ERROR_TRAVEL_DISTANCE start microswitch must be off
        MicroSwitchStartValue = digitalRead(MicroSwitchStartPin);
  
        if (MicroSwitchStartValue == LOW)
        { // Start Microswitch pushed - Error - Stop platform now
          Set_Error("Err5");
        }
      }
      if (travelDistance > total_travel_distance+END_ERROR_TRAVEL_DISTANCE)
      { // Stop micro switch should have been pushed by now - Error - Stop platform now
          Set_Error("Err6");
      }
      if ((remainingDistanceDone == 1) && (travelDistance > END_ERROR_TRAVEL_DISTANCE))
      {
        // Stop micro switch should have been pushed by now - Error - Stop platform now
          Set_Error("Err7");
      }
    }

    // Print passed time
    if ((finished_ms - timeout_ms) >= DISPLAY_TIMEOUT)
    { // Update display only after timeout
      if (DisplayModeButton == 1)
      { // Button is pressed - change display mode
        DisplayModeButton = 0;
        DisplayMode++;
        if (DisplayMode == MAX_D)
        {
          DisplayMode = PERCENTAGE_D;
        }
      }

      if ((DisplayMode == TIME_D) || (remainingDistance == 0))
      {
        long delta_min = delta_sec/60;
        delta_sec = delta_sec % 60;
        display.showNumberDec(delta_min*100+delta_sec,0b01000000, 1);
      }
      else if (DisplayMode == PERCENTAGE_D)
      {
          display.showString("%", 1, 3);   
          display.showNumber((int)percentage,false, 3, 0);
      }
      else // DisplayMode == BAR_D
      {
          display.showLevel(5+(int)percentage, false);
      }
    }

    elapsed_ms = finished_ms;
  }
}

////////////////////////////////////////////////////////////////////////////////
int print2serial = 0;
void do_running_loop(void)
{
  //stepper.runSpeed(); // Activate motor
  float lastRPM = targetRPM;

  // First - test for stop microswitch
  MicroSwitchStopValue = digitalRead(MicroSwitchStopPin);
  
  if (MicroSwitchStopValue == LOW)
  { // Stop Microswitch pushed
    buttonStopCount++;
    if(buttonStopCount == buttonCount)
    {  // Microswitch stop was met - need to reset platform
      remainingDistance = total_travel_distance;
      remainingDistanceDone = 0;
      if (Return_policy == AUTO_RT)
      {
        Platform_state = RETURN_ST;
        Timer1.initialize(TIMER_FAST_PERIOD);
        Timer1.start();
        DisplayModeButton = 0;
        targetSpeed = (returnRPM * stepsPerRevolution) / 60.0;
        timeout_ms = millis();
        stepper.setSpeed(targetSpeed);
        display.showString("rtrn");        
        Serial.println("Motor reversed");
      }
      else
      { // (Return_policy == MANUAL_RT) || (Return_policy == MEASURE_RT)
        digitalWrite (enablePin, HIGH); // Disable motor
        Serial.println("Motor disabled");
        Platform_state = WAIT_RETURN_ST;
        Timer1.stop();
        if (Return_policy == MEASURE_RT)
        {
          PrintTravelDistance();
        }
        else
        {
          display.showString("push");
        }
      }
      buttonStopCount = 0;
    }
  }
  else
  { // Button released
    buttonStopCount = 0;
  }

  if (digitalRead(resetPin) == LOW)
  {
      digitalWrite (enablePin, HIGH); // Disable motor
      Serial.println("Motor disabled");
      Platform_state = STOP_ST;
      Timer1.stop();
      display.showString("rdy");
  }
  else
  {
    // Now - handle speed changes
    //stepper.runSpeed(); // Activate motor
    if (read_speed_buttons() == LOW)
    { // Button is pressed
      if (!((buttonSpeedDownValue == LOW) && (buttonSpeedUpValue == LOW)))
      { // Only one button is pressed (disregard when both are pressed)
        if (buttonSpeedUpValue == LOW)
        { // Increase speed
          if (fast_forward_ms == 0)
          {
            original_angular_velocity = angular_velocity;
            fast_forward_ms = millis();
          }
          fast_backward_ms = 0;
          angular_velocity += ANGULAR_VELOCITY_STEP;
        }
        else
        { // Decrease speed
          if (fast_backward_ms == 0)
          {
            original_angular_velocity = angular_velocity;
            fast_backward_ms = millis();
          }
          angular_velocity -= ANGULAR_VELOCITY_STEP;
          fast_forward_ms = 0;
        }
        //half_arch_travel_distance = push_arch_radius*atan(TRAVEL_DISTANCE/2/push_arch_radius);
        nominalRPM = (angular_velocity*push_arch_radius*(float)ROD_PINS/(float)MOTOR_PINS);
        nominalRPM_offset = (angular_velocity*RADIUS_OFFSET*(float)ROD_PINS/(float)MOTOR_PINS);
        FixTargetRPMAccordingToLoacation();

        timeout_ms = millis(); // This will make the speed appear on the screen for 3 seconds
        Serial.print("New angular velocity: ");
        Serial.println(angular_velocity*1000000.0);
        print2serial = 1;
        if (tracking_rate_mode == SIDEREAL_RATE)
        {
          display.showNumber((angular_velocity-SIDEREAL_ANGULAR_VELOCITY)*1000000.0, 0);
        }
        else
        { // MOON_RATE
          display.showNumber((angular_velocity-MOON_ANGULAR_VELOCITY)*1000000.0, 0);
        }
        // Calculate the desired speed based on RPM
        targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;

        // Set the desired speed
        stepper.setSpeed(targetSpeed);
      }
      else
      { // Both buttons are pressed - check for fast forwarding or fast backward
        if ((fast_forward_ms > 0) && (millis()-fast_forward_ms >= RESET_BUTTON_DELAY))
        {
          // Fast forwarding requested
          Platform_state = FAST_FORWARD_ST;
          Timer1.initialize(TIMER_FAST_PERIOD);
          Timer1.start();
          DisplayModeButton = 0;
          targetRPM = -returnRPM;
          targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;
          timeout_ms = millis();
          start_ms = timeout_ms;
          stepper.setSpeed(targetSpeed);
          display.showString("ff");
          tone(buzzerPin,BUZZER_FREQ1,BUZZER_DURATION);
        }
        if ((fast_backward_ms > 0) && (millis()-fast_backward_ms >= RESET_BUTTON_DELAY))
        {
          // Fast backward requested
          if (remainingDistance > 0)
          {
            remainingDistance = total_travel_distance - remainingDistance;
          }
          Platform_state = RETURN_ST;
          Timer1.initialize(TIMER_FAST_PERIOD);
          Timer1.start();
          DisplayModeButton = 0;
          targetRPM = returnRPM;
          angular_velocity = original_angular_velocity;
          nominalRPM = (angular_velocity*push_arch_radius*(float)ROD_PINS/(float)MOTOR_PINS);
          nominalRPM_offset = (angular_velocity*RADIUS_OFFSET*(float)ROD_PINS/(float)MOTOR_PINS);
          targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;
          timeout_ms = millis();
          start_ms = timeout_ms;
          stepper.setSpeed(targetSpeed);
          display.showString("rtrn");        
          Serial.println("Motor reversed");
        }
      }
    }
    else
    {
      if (buttonPressedTime_ms == 0)
      {
        fast_forward_ms = 0;
        fast_backward_ms = 0;
      }
    }
  }

  finished_ms = millis();
  
  // Print time
  if ((finished_ms - elapsed_ms >= DISPLAY_DELTA) && (Platform_state == RUNNING_ST))
  { // One second has passed. Update display
    //stepper.runSpeed(); // Activate motor
    long delta_sec = (finished_ms-start_ms)/1000;
    if (remainingDistance > 0)
    { // Print remaining time
      remainingDistance = remainingDistance - (lastRPM * (float)MOTOR_PINS/ROD_PINS * (float)(finished_ms - elapsed_ms)/60000);
      if (remainingDistance<0)
      {
        remainingDistance = 0;
        remainingDistanceDone = 1;
        travelDistance = 0;
        delta_sec = 0;
        start_ms = finished_ms;
      }
      else
      { // Fix targetRPM according to location
//        Serial.print ("remainingDistance ");
  //      Serial.println(remainingDistance);
    //    float distanceRatio = (remainingDistance-(total_travel_distance/2))/push_arch_radius; // tan(alpha)
//        Serial.print ("distanceRatio ");
//        Serial.println(distanceRatio);
        //float alpha = atan(distanceRatio);
//        Serial.print ("alpha ")
//        Serial.println(alpha);
        //float ArchRemainingDistance = push_arch_radius*alpha + half_arch_travel_distance;
//        Serial.print ("ArchRemainingDistance ");
//        Serial.println(ArchRemainingDistance);

        //float remainingTime = ArchRemainingDistance / (nominalRPM * (float)MOTOR_PINS/ROD_PINS)*60.0;
        float remainingTime = remainingDistance / (nominalRPM * (float)MOTOR_PINS/ROD_PINS)*60.0;
//        Serial.print ("remainingTime ");
//        Serial.println(remainingTime);
        //float remainingTime = remainingDistance / (nominalRPM * (float)MOTOR_PINS/ROD_PINS)*60*SIN_APPROXIMATION_FACTOR;
        delta_sec = remainingTime;
      //  targetRPM = nominalRPM*cos(distanceRatio*CORRECTION_FACTOR);
        //targetRPM = nominalRPM*cos(alpha*ANGLE_CORRECTION_FACTOR);
        FixTargetRPMAccordingToLoacation();
        if (print2serial == 1)
        {
          print2serial = 0;
          Serial.print ("remainingDistance ");
          Serial.print(remainingDistance);
          Serial.print("   targetRPM: ");
          Serial.println(targetRPM,4);
        }
        targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;
        stepper.setSpeed(targetSpeed);

        // Buzzer if remaining time is 5 minutes (or less)
        if ((buzzerOn == 0) && (remainingTime <= BUZZER_TIME1+1))
        {
          tone(buzzerPin,BUZZER_FREQ1,BUZZER_DURATION);
          buzzerOn = 1;
        }
        else if ((buzzerOn == 1) && (remainingTime <= BUZZER_TIME2+1))
        {
          tone(buzzerPin,BUZZER_FREQ2,BUZZER_DURATION);
          buzzerOn = 2;
        }
        else if ((buzzerOn == 2) && (remainingTime <= BUZZER_TIME3+1))
        {
          tone(buzzerPin,BUZZER_FREQ3,BUZZER_DURATION);
          buzzerOn = 3;
        }
      }


      // Test for error condition - Start microswitch should be off after some time
      if (total_travel_distance - remainingDistance > ERROR_TRAVEL_DISTANCE)
      { // After ERROR_TRAVEL_DISTANCE start microswitch must be off
        MicroSwitchStartValue = digitalRead(MicroSwitchStartPin);
  
        if (MicroSwitchStartValue == LOW)
        { // Start Microswitch pushed - Error - Stop platform now
          Set_Error("Err1");
        }
      }
    }
    else
    {  // Calculate travel distance
      travelDistance = travelDistance + (lastRPM * (float)MOTOR_PINS/ROD_PINS * (float)(finished_ms - elapsed_ms)/60000);
      if (travelDistance > ERROR_TRAVEL_DISTANCE)
      { // After ERROR_TRAVEL_DISTANCE start microswitch must be off
        MicroSwitchStartValue = digitalRead(MicroSwitchStartPin);
  
        if (MicroSwitchStartValue == LOW)
        { // Start Microswitch pushed - Error - Stop platform now
          Set_Error("Err2");
        }
      }
      if (travelDistance > total_travel_distance+ERROR_TRAVEL_DISTANCE)
      { // Stop micro switch should have been pushed by now - Error - Stop platform now
          Set_Error("Err3");
      }
      if ((remainingDistanceDone == 1) && (travelDistance > ERROR_TRAVEL_DISTANCE))
      {
        // Stop micro switch should have been pushed by now - Error - Stop platform now
          Set_Error("Err8");
      }
    }

    // Print passed time only after timeout
    if ((finished_ms - timeout_ms) >= DISPLAY_TIMEOUT) 
    {
      long delta_min = delta_sec/60;
      delta_sec = delta_sec % 60;
      if (Return_policy == MEASURE_RT)
      {
        PrintTravelDistance();
      }
      else
      {
  //        display.showNumber(targetRPM, 3);
        display.showNumberDec(delta_min*100+delta_sec,0b01000000, 1);
      }
    }
    elapsed_ms = finished_ms;
  }
}

////////////////////////////////////////////////////////////////////////////////

void do_return_loop(void)
{
  //stepper.runSpeed(); // Activate motor
  noTone(buzzerPin); // Just to be sure

  // Test for start microswitch
  MicroSwitchStartValue = digitalRead(MicroSwitchStartPin);
  
  if (MicroSwitchStartValue == LOW)
  { // Start Microswitch pushed
    buttonStartCount++;
    if(buttonStartCount == buttonCount)
    {  // Microswitch Start was met - restart the motor
      if (targetSpeed <= 0.0)
      {
        if (Return_policy == MEASURE_RT)
        {
          remainingDistance = 0;
          travelDistance = 0;
        }
        else
        {
          remainingDistance = total_travel_distance;
          FixTargetRPMAccordingToLoacation(); // Update targetRPM
        }
        remainingDistanceDone = 0;
        targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;
        stepper.setSpeed(targetSpeed);
        display.showString("Strt");
        timeout_ms = millis();
        start_ms = timeout_ms;
        Platform_state = RUNNING_ST;
        Timer1.initialize(TIMER_SLOW_PERIOD);
        Timer1.start();
        buzzerOn = 0;
        Serial.println("Motor restarted");
      }
      buttonStartCount = 0;
    }
  }

  if (digitalRead(resetPin) == LOW)
  {
      digitalWrite (enablePin, HIGH); // Disable motor
      if (remainingDistance > 0)
      {
        remainingDistance = total_travel_distance - remainingDistance;
        FixTargetRPMAccordingToLoacation(); // Update targetRPM
      }
      else
      {
        targetRPM = nominalRPM;
      }
      targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;
      stepper.setSpeed(targetSpeed);
      Platform_state = STOP_ST;
      Timer1.stop();
      display.showString("rdy");
  }
  

  finished_ms = millis();

  if ((finished_ms - timeout_ms) >= DISPLAY_TIMEOUT)
  { // Disregard keys pressed before timeout
    if (read_speed_buttons() == LOW)
    {
      DisplayModeButton = 1;
    }
  }

  // Print time after timeout
  if (finished_ms - elapsed_ms >= DISPLAY_DELTA)
  { // One second has passed. Update display
    float percentage = 0;
    long delta_sec = (finished_ms-start_ms)/1000;
    if (remainingDistance > 0)
    { // Print remaining time
      remainingDistance = remainingDistance - (-returnRPM * (float)MOTOR_PINS/ROD_PINS * (float)(finished_ms - elapsed_ms)/60000);
      percentage = (remainingDistance / total_travel_distance) * 100.0;
      float remainingTime = remainingDistance / (-returnRPM * (float)MOTOR_PINS/ROD_PINS)*60;
      delta_sec = remainingTime;
      if (remainingDistance<0)
      {
        remainingDistance = 0;
        remainingDistanceDone = 1;
        travelDistance = 0;
        delta_sec = 0;
        start_ms = finished_ms;
      }
      // Test for error condition - Stop microswitch should be off after some time
      if (total_travel_distance - remainingDistance > ERROR_TRAVEL_DISTANCE)
      { // After ERROR_TRAVEL_DISTANCE stop microswitch must be off
        MicroSwitchStopValue = digitalRead(MicroSwitchStopPin);
  
        if (MicroSwitchStopValue == LOW)
        { // Start Microswitch pushed - Error - Stop platform now
          Set_Error("Err9");
        }
      }
    }
    else
    {  // Calculate travel distance
      travelDistance = travelDistance + (-returnRPM * (float)MOTOR_PINS/ROD_PINS * (float)(finished_ms - elapsed_ms)/60000);
      if (travelDistance > ERROR_TRAVEL_DISTANCE)
      { // After ERROR_TRAVEL_DISTANCE Stop microswitch must be off
        MicroSwitchStopValue = digitalRead(MicroSwitchStopPin);
  
        if (MicroSwitchStopValue == LOW)
        { // Stop Microswitch pushed - Error - Stop platform now
          Set_Error("Er11");
        }
      }
      if (travelDistance > total_travel_distance+ERROR_TRAVEL_DISTANCE)
      { // Start micro switch should have been pushed by now - Error - Stop platform now
          Set_Error("Er12");
      }
      if ((remainingDistanceDone == 1) && (travelDistance > ERROR_TRAVEL_DISTANCE))
      {
        // Start micro switch should have been pushed by now - Error - Stop platform now
          Set_Error("Err0");
      }
    }

    // Print passed time
    if ((finished_ms - timeout_ms) >= DISPLAY_TIMEOUT)
    { // Update display only after timeout
      if (DisplayModeButton == 1)
      { // Button is pressed - change display mode
        DisplayModeButton = 0;
        DisplayMode++;
        if (DisplayMode == MAX_D)
        {
          DisplayMode = PERCENTAGE_D;
        }
      }

      if ((DisplayMode == TIME_D) || (remainingDistance == 0))
      {
        long delta_min = delta_sec/60;
        delta_sec = delta_sec % 60;
        display.showNumberDec(delta_min*100+delta_sec,0b01000000, 1);
      }
      else if (DisplayMode == PERCENTAGE_D)
      {
          display.showString("%", 1, 3);   
          display.showNumber((int)percentage,false, 3, 0);
      }
      else // DisplayMode == BAR_D
      {
          display.showLevel(5+(int)percentage, false);
      }
    }
    elapsed_ms = finished_ms;
  }
}

////////////////////////////////////////////////////////////////////////////////

void do_stop_loop(void)
{
  noTone(buzzerPin); // Just to be sure

  if (read_speed_buttons() == LOW)
  { // Button is pressed
    digitalWrite (enablePin, LOW); // Enable motor
    tone(buzzerPin,BUZZER_FREQ1,BUZZER_DURATION);
    Platform_state = RUNNING_ST;
    Timer1.initialize(TIMER_SLOW_PERIOD);
    Timer1.start();

    buzzerOn = 0;
    Serial.println("Motor enabled");
    display.showString("Strt");
    timeout_ms = millis();
    elapsed_ms = timeout_ms;
    start_ms = timeout_ms;
  }

  if (digitalRead(resetPin) == LOW)
  {
    if (reset_ms == 0)
    {
      reset_ms = millis();
    }
    else if (millis()-reset_ms >= RESET_BUTTON_DELAY)
    { // Change nominal tracking mode
      if (reset_button_pushed == 0)
      {
        reset_button_pushed = 1;
        if (tracking_rate_mode == SIDEREAL_RATE)
        { // Switch to Moon mode
          angular_velocity = MOON_ANGULAR_VELOCITY;
          tracking_rate_mode = MOON_RATE;
          Serial.println("Moon");
          display.showString("Luna");
        }
        else if (tracking_rate_mode == MOON_RATE)
#ifdef SUN_RATE_SUPPORT      
        { // Switch to Sun mode
          angular_velocity = SUN_ANGULAR_VELOCITY;
          tracking_rate_mode = SUN_RATE;
          Serial.println("Sun");
          display.showString("Sun");
        }
        else // (tracking_rate_mode == SUN_RATE)
#endif        
        { // Switch to Sidereal mode
          angular_velocity = SIDEREAL_ANGULAR_VELOCITY;
          tracking_rate_mode = SIDEREAL_RATE;
          Serial.println("Sidereal");
          display.showString("Side");
        }
        nominalRPM = (angular_velocity*push_arch_radius*(float)ROD_PINS/(float)MOTOR_PINS);
        nominalRPM_offset = (angular_velocity*RADIUS_OFFSET*(float)ROD_PINS/(float)MOTOR_PINS);
        FixTargetRPMAccordingToLoacation();
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void do_wait_return_loop(void)
{
  noTone(buzzerPin); // Just to be sure

  if (read_speed_buttons() == LOW)
  { // Button is pressed - enable motor
    digitalWrite (enablePin, LOW); // Enable motor
    tone(buzzerPin,BUZZER_FREQ1,BUZZER_DURATION);
    remainingDistance = total_travel_distance;
    FixTargetRPMAccordingToLoacation();
    remainingDistanceDone = 0;
    Platform_state = RETURN_ST;
    Timer1.initialize(TIMER_FAST_PERIOD);
    Timer1.start();
    DisplayModeButton = 0;
    targetSpeed = (returnRPM * stepsPerRevolution) / 60.0;
    if (Return_policy == MEASURE_RT)
    {
      PrintTravelDistance();
    }
    else
    {
      display.showString("rtrn");        
    }
    timeout_ms = millis();
    elapsed_ms = timeout_ms;
    start_ms = timeout_ms;
    stepper.setSpeed(targetSpeed);
    Serial.println("Motor reversed");
  }
}

////////////////////////////////////////////////////////////////////////////////

void loop()
{  
  switch(Platform_state)
  {
    case SETUP_ST:
      do_setup_loop();
      break;
    case RUNNING_ST:
      do_running_loop();
      break;
    case FAST_FORWARD_ST:
      do_fast_forward_loop();
      break;
    case RETURN_ST:
      do_return_loop();
      break;
    case WAIT_RETURN_ST:
      do_wait_return_loop();
      break;
    case STOP_ST:
      do_stop_loop();
      break;
  }
  
#if 0  
  // Change remaining distance with serial input
  if (Serial.available())
  {
    // Read the serial input.
    float new_remainingDistance = Serial.parseFloat();
    if (new_remainingDistance != 0)
    {
      remainingDistance = new_remainingDistance;
    }
  }
#endif

  if (digitalRead(resetPin) == LOW)
  {
    if (reset_ms == 0)
    {
      reset_ms = millis();
    }
    else if (millis()-reset_ms >= 2*RESET_BUTTON_DELAY)
    { // Reset requested
      Platform_state = SETUP_ST;
      Timer1.stop();
      init_vars();
      reset_button_pushed = 1;
    }
  }
  else
  {
    reset_button_pushed = 0;
    reset_ms = 0;
  }
}
