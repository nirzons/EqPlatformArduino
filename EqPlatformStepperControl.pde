#include <AccelStepper.h>
//#include <TM1637TinyDisplayPrivate.h>
#include <TM1637TinyDisplay.h>
#include <TimerOne.h>
// Version 8 - Modify manual mode. Add fast forwarding

void(* resetFunc) (void) = 0;//declare reset function at address 0

// The distance between the start and stop position of the platform in mm
#define TRAVEL_DISTANCE 180.3 // Nir
//#define TRAVEL_DISTANCE 176.0 // Guy

// Number of pins on the motor screw
#define MOTOR_PINS 12
// Number of pins on the rod screw
#define ROD_PINS 30

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
#define DISPLAY_DELTA (1000) // 1 second
#define BUZZER_DURATION (40) // 40ms
#define BUZZER_FREQ1 (400)
#define BUZZER_TIME1 (5*60) // 5 minutes
#define BUZZER_FREQ2 (600)
#define BUZZER_TIME2 (1*60) // 1 minutes
#define BUZZER_FREQ3 (800)
#define BUZZER_TIME3 (15) // 15 seconds
#define NOMINAL_RPM (6.90)
#define RETURN_RPM (-250.0)
#define RETURN_RPM_STEP (90.0)
#define FAST_FORWARD_RPM (150.0)
#define SETUP_TIMEOUT (8000) // 8 seconds
#define SETUP_COUNTDOWN_DELAY (3000) // three seconds
#define RESET_BUTTON_DELAY (2000) // two seconds

typedef enum {
  AUTO_RT = 0,
  MANUAL_RT
} Return_policy_e;

// Defaut return policity
#define DEFAULT_RETURN AUTO_RT
//#define DEFAULT_RETURN MANUAL_RT

typedef enum {
  CONST_SPEED_ST = 0,
  VARIABLE_SPEED_ST
} state_e;

// Default operation state
#define DEFAULT_STATE CONST_SPEED_ST

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


Return_policy_e Return_policy;
Platform_state_e Platform_state;

// Create a display object of type TM1637TinyDisplay 4 digit display
TM1637TinyDisplay display(CLK, DIO);

//float stepsPerRevolution = 6400.0;
float stepsPerRevolution = 800.0;

 // Set the desired rotation speed in RPM
float Original_targetRPM;
float targetRPM;
float returnRPM;
int returnRPM_setup;

// remaining distance till arriving to stop microswitch
float remainingDistance;

AccelStepper stepper=AccelStepper(Interface, stepPin, dirPin); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

// Calculate the desired speed based on RPM
float targetSpeed;
float MaxSpeed;

unsigned long buttonPressedTime_ms;
int buttonStartCount;
int buttonStopCount;
int buttonSpeedPushed;

int buzzerOn;

int EECounter;

unsigned long start_ms, finished_ms, elapsed_ms, timeout_ms, setup_countdown_ms, reset_ms, fast_forward_ms;

void init_vars()
{
  digitalWrite (enablePin, HIGH); // Disable motor

  buzzerOn = 0;
  noTone(buzzerPin); // Just to be sure

  // Set the desired rotation speed in RPM
  targetRPM = NOMINAL_RPM;
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

  Return_policy = DEFAULT_RETURN;

  remainingDistance = 0;

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

  Timer1.initialize(150);
  Timer1.attachInterrupt(runSpeedHook); // runSpeedHook to run every 0.15 miliseconds
  Timer1.start();
  
  init_vars();
}

  int MicroSwitchStartValue;
  int MicroSwitchStopValue;
  int buttonSpeedUpValue;
  int buttonSpeedDownValue;


////////////////////////////////////////////////////////////////////////////////
// This hook is called from within the private version TM1637TinyDisplay
// so that the motor does not stop while writing to the display
void runSpeedHook(void)
{
 if ((Platform_state == RUNNING_ST) || (Platform_state == RETURN_ST) || (Platform_state == FAST_FORWARD_ST))
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

  if (read_speed_buttons() == LOW)
  { // Button is pressed
    elapsed_ms = millis();
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
        if (Return_policy == AUTO_RT)
        {
          Return_policy = MANUAL_RT;
          Serial.println("Hand");
          display.showString("HAnd");
        }
        else
        {
          Return_policy = AUTO_RT;
          Serial.println("Auto");
          display.showString("Auto");
        }
      }
      else
      { 
        if (Return_policy == AUTO_RT)
        { // Speed down button - change return speed - only for auto return policy
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

  finished_ms = millis();
  // Test setup timeout
  if ((finished_ms-elapsed_ms) > SETUP_TIMEOUT)
  { // Setup timeout expired - finish setup
    Platform_state = STOP_ST;
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
        targetRPM = Original_targetRPM; // Resume normal operation
        targetSpeed = (returnRPM * stepsPerRevolution) / 60.0;
        stepper.setSpeed(targetSpeed);
        Platform_state = WAIT_RETURN_ST;
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
      targetRPM = Original_targetRPM; // Resume normal operation
      targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;
      stepper.setSpeed(targetSpeed);
      Platform_state = STOP_ST;
      display.showString("rdy");
  }

  finished_ms = millis();
  
  // Print time after timeout
  if (((finished_ms - timeout_ms) >= DISPLAY_TIMEOUT) && (finished_ms - elapsed_ms >= DISPLAY_DELTA))
  { // One second has passed. Update display
    //stepper.runSpeed(); // Activate motor
    long delta_sec = (finished_ms-start_ms)/1000;
    if (remainingDistance > 0)
    { // Print remaining time
      remainingDistance = remainingDistance - (lastRPM * (float)MOTOR_PINS/ROD_PINS * (float)(finished_ms - elapsed_ms)/60000);
      float remainingTime = remainingDistance / (targetRPM * (float)MOTOR_PINS/ROD_PINS)*60;
      delta_sec = remainingTime;
      if (remainingDistance<0)
      {
        remainingDistance = 0;
        delta_sec = 0;
        start_ms = finished_ms;
      }
    }


    // Print passed time
    long delta_min = delta_sec/60;
    delta_sec = delta_sec % 60;
    //stepper.runSpeed(); // Activate motor
  	display.showNumberDec(delta_min*100+delta_sec,0b01000000, 1);

    //stepper.runSpeed(); // Activate motor
  	//display.showNumber((long)(finished_ms-start_ms)/1000);
    //  Serial.println(finished_ms-start_ms);
    elapsed_ms = finished_ms;
  }
}

////////////////////////////////////////////////////////////////////////////////

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
      if (Return_policy == AUTO_RT)
      {
        Platform_state = RETURN_ST;
        targetSpeed = (returnRPM * stepsPerRevolution) / 60.0;
        remainingDistance = TRAVEL_DISTANCE;
        timeout_ms = millis();
        stepper.setSpeed(targetSpeed);
        display.showString("rtrn");        
        Serial.println("Motor reversed");
      }
      else
      { // (Return_policy == MANUAL_RT)
        digitalWrite (enablePin, HIGH); // Disable motor
        Serial.println("Motor disabled");
        Platform_state = WAIT_RETURN_ST;
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
      Serial.println("Motor disabled");
      Platform_state = STOP_ST;
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
            Original_targetRPM = targetRPM;
            fast_forward_ms = millis();
          }
          targetRPM += 0.02;
        }
        else
        { // Decrease speed
          fast_forward_ms = 0;
          targetRPM -= 0.02;
        }
        timeout_ms = millis(); // This will make the speed apear of the screen for 3 seconds
        Serial.print("New speed: ");
        Serial.println(targetRPM);
        // Calculate the desired speed based on RPM
        display.showNumber(targetRPM, 2);
        targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;

        // Set the desired speed
        stepper.setSpeed(targetSpeed);
      }
      else
      { // Both buttons are pressed - check for fast forwarding
        if ((fast_forward_ms > 0) && (millis()-fast_forward_ms >= RESET_BUTTON_DELAY))
        {
          // Fast forwarding requested
          Platform_state = FAST_FORWARD_ST;
          targetRPM = -returnRPM;
          targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;
          timeout_ms = millis();
          stepper.setSpeed(targetSpeed);
          display.showString("ff");
          tone(buzzerPin,BUZZER_FREQ1,BUZZER_DURATION);
        }
      }
    }
    else
    {
      if (buttonPressedTime_ms == 0)
      {
        fast_forward_ms = 0;
      }
    }
  }

#if 0
  // Print RPM after timeout
  finished_ms = millis();
  if ((timeout_ms > 0) && ((finished_ms - timeout_ms) >= DISPLAY_TIMEOUT))
  {
    display.showNumber(targetRPM, 1);
    timeout_ms = 0;
  }
#else
  finished_ms = millis();
  
  // Print time after timeout
  if (((finished_ms - timeout_ms) >= DISPLAY_TIMEOUT) && (finished_ms - elapsed_ms >= DISPLAY_DELTA))
  { // One second has passed. Update display
    //stepper.runSpeed(); // Activate motor
    long delta_sec = (finished_ms-start_ms)/1000;
    if (remainingDistance > 0)
    { // Print remaining time
      remainingDistance = remainingDistance - (lastRPM * (float)MOTOR_PINS/ROD_PINS * (float)(finished_ms - elapsed_ms)/60000);
      float remainingTime = remainingDistance / (targetRPM * (float)MOTOR_PINS/ROD_PINS)*60;
      delta_sec = remainingTime;
      if (remainingDistance<0)
      {
        remainingDistance = 0;
        delta_sec = 0;
        start_ms = finished_ms;
      }

      // Buzzer if remaining time is 5 minutes (or less)
      else if ((buzzerOn == 0) && (remainingTime <= BUZZER_TIME1+1))
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


    // Print passed time
    long delta_min = delta_sec/60;
    delta_sec = delta_sec % 60;
    //stepper.runSpeed(); // Activate motor
  	display.showNumberDec(delta_min*100+delta_sec,0b01000000, 1);

    //stepper.runSpeed(); // Activate motor
  	//display.showNumber((long)(finished_ms-start_ms)/1000);
    //  Serial.println(finished_ms-start_ms);
    elapsed_ms = finished_ms;
  }
#endif
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
        remainingDistance = TRAVEL_DISTANCE;
        targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;
        stepper.setSpeed(targetSpeed);
        display.showString("Strt");
        timeout_ms = millis();
        Platform_state = RUNNING_ST;
        buzzerOn = 0;
        Serial.println("Motor restarted");
      }
      buttonStartCount = 0;
    }
  }

  finished_ms = millis();
  // Print time after timeout
  if (((finished_ms - timeout_ms) >= DISPLAY_TIMEOUT) && (finished_ms - elapsed_ms >= DISPLAY_DELTA))
  { // One second has passed. Update display
    //stepper.runSpeed(); // Activate motor
    long delta_sec = (finished_ms-start_ms)/1000;
    if (remainingDistance > 0)
    { // Print remaining time
      remainingDistance = remainingDistance - (-returnRPM * (float)MOTOR_PINS/ROD_PINS * (float)(finished_ms - elapsed_ms)/60000);
      float remainingTime = remainingDistance / (-returnRPM * (float)MOTOR_PINS/ROD_PINS)*60;
      delta_sec = remainingTime;
      if (remainingDistance<0)
      {
        remainingDistance = 0;
        delta_sec = 0;
        start_ms = finished_ms;
      }
    }

    // Print passed time
    long delta_min = delta_sec/60;
    delta_sec = delta_sec % 60;
    //stepper.runSpeed(); // Activate motor
  	display.showNumberDec(delta_min*100+delta_sec,0b01000000, 1);
    //stepper.runSpeed(); // Activate motor
  	//display.showNumber((long)(finished_ms-start_ms)/1000);
    //  Serial.println(finished_ms-start_ms);
    elapsed_ms = finished_ms;
  }
}

////////////////////////////////////////////////////////////////////////////////

void do_stop_loop(void)
{
  noTone(buzzerPin); // Just to be sure

  if (read_speed_buttons() == LOW)
  { // Button is pressed - enable motor
    digitalWrite (enablePin, LOW); // Enable motor
    tone(buzzerPin,BUZZER_FREQ1,BUZZER_DURATION);
    Platform_state = RUNNING_ST;
    buzzerOn = 0;
    Serial.println("Motor enabled");
    display.showString("Strt");
    timeout_ms = millis();
    start_ms = timeout_ms;
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
    Platform_state = RETURN_ST;
    targetSpeed = (returnRPM * stepsPerRevolution) / 60.0;
    remainingDistance = TRAVEL_DISTANCE;
    timeout_ms = millis();
    stepper.setSpeed(targetSpeed);
    display.showString("rtrn");        
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
  
  // Change speed with serial input
  if (Serial.available())
  {
    // Read the serial input.
    float newestSpeed = Serial.parseFloat();
    if (newestSpeed != 0)
    {
      targetRPM = newestSpeed;
      Serial.print("New speed: ");
      Serial.println(targetRPM);
      // Calculate the desired speed based on RPM
      display.showNumber(targetRPM, 1);
      targetSpeed = (targetRPM * stepsPerRevolution) / 60.0;

      // Set the desired speed
      stepper.setSpeed(targetSpeed);
    }
  }

  if (digitalRead(resetPin) == LOW)
  {
    if (reset_ms == 0)
    {
      reset_ms = millis();
    }
    else if (millis()-reset_ms >= RESET_BUTTON_DELAY)
    {
      // Reset requested
      Platform_state = SETUP_ST;
      init_vars();
    }
  }
  else
  {
    reset_ms = 0;
  }
}
