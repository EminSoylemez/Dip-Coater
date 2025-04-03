#include <AccelStepper.h>
#include <SoftwareSerial.h>
#include <FastLED.h>
#include <string.h>
SoftwareSerial Serial2(3,4); // RX,TX
#include "Countimer.h"
Countimer DippingTimer;
Countimer LiftingTimer;
Countimer TrayTimer;

#define LED_PIN A0
#define NUM_LEDS A1
#define BUZZER_PIN 5
#define HALL_SENSOR 6
#define TRAY_HALL_SENSOR 7
CRGB leds[NUM_LEDS];

int EN_1 = 8;
int EN_2 = 9;
int STEP_1 = 10;
int STEP_2 = 11;
int DIR_1 = 12;
int DIR_2 = 13;

AccelStepper myStepper(1, STEP_1, DIR_1); //Number of Motors,StepPin,DirPin
AccelStepper myStepper_2(1, STEP_2, DIR_2);

const byte numChars = 200;
char receivedChars[numChars];
char tempChars[numChars];
int valuesCount = 0;
bool dataarrived=0;
float Data[74]={0.0};
boolean newData = false;
long Homing=-1;
long TrayHoming=-1;
int Cycles=0;
bool Dipped=1;
bool Withdraw=1;
int count=0;
char global_rc;
int total_cycle_count = 1;
int currentPosition = 0;

// State machine variables
enum SystemState {
  IDLE,
  DIPPING,
  LIFTING,
  TRAY_WAITING,
  TRAY_MOVING,
  HOMING,
  FINISHING
};

SystemState currentState = IDLE;
unsigned long stateStartTime = 0;
bool operationInProgress = false;
bool buzzerEnabled = false;

// Motion targets
long dippingTarget = 0;
long liftingTarget = 0;
long trayTarget = 0;
unsigned long waitTime = 0;

void setup() {
  pinMode(EN_1, OUTPUT);
  digitalWrite(EN_1, LOW);
  pinMode(EN_2, OUTPUT);
  digitalWrite(EN_2, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  //FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  pinMode(HALL_SENSOR, INPUT);
  pinMode(TRAY_HALL_SENSOR, INPUT);
  
  Serial.begin(9600);
  Serial2.begin(9600);
  
  DippingTimer.setInterval(DippingTimeDisp, 1000);
  LiftingTimer.setInterval(LiftingTimeDisp, 1000);
  TrayTimer.setInterval(TrayTimeDisp, 1000);
  
  homefunction();
}

void loop() {
  // Always run timers
  DippingTimer.run();
  LiftingTimer.run();
  TrayTimer.run();
  
  // Always check for serial commands
  recvWithStartEndMarkers();
  if (newData == true) {
    memcpy(tempChars, receivedChars, numChars * sizeof(char));
    parseData();
    showParsedData();
    resetSystem();
    newData = false;
  }
  
  // State machine handling
  switch (currentState) {
    case IDLE:
      // Check if we should start a new operation
      if (!operationInProgress) {
        checkStartOperation();
      }
      break;
      
    case DIPPING:
      handleDippingState();
      break;
      
    case LIFTING:
      handleLiftingState();
      break;
      
    case TRAY_WAITING:
      handleTrayWaitingState();
      break;
      
    case TRAY_MOVING:
      handleTrayMovingState();
      break;
      
    case HOMING:
      handleHomingState();
      break;
      
    case FINISHING:
      handleFinishingState();
      break;
  }
  
  // Always run motors to handle any pending movements
  myStepper.run();
  myStepper_2.run();
}

void resetSystem() {
  Cycles = 0;
  count = 0;
  Dipped = 1;
  Withdraw = 1;
  currentState = IDLE;
  operationInProgress = false;
  currentPosition = myStepper.currentPosition();
  
  DippingTimer.restart();
  LiftingTimer.restart();
  TrayTimer.restart();
  DippingTimer.pause();
  LiftingTimer.pause();
  TrayTimer.pause();
  
  Serial2.print("n2.val=1\xFF\xFF\xFF");
}

void checkStartOperation() {
  // Check for valid data to start a simulation
  int t = getSimulationOffset();
  if (t >= 0 && Data[5+t] > 0 && Data[6+t] > 0 && Data[7+t] > 0 && 
      Data[8+t] > 0 && Data[9+t] > 0 && Data[5+t] <= 20 && Data[8+t] <= 20) {
    
    Serial2.print("page 4\xFF\xFF\xFF");
    updateLEDs(0, 255, 0);
    
    // Set up timers
    DippingTimer.setCounter(0, 0, Data[7+t], DippingTimer.COUNT_DOWN, onComplete);
    LiftingTimer.setCounter(0, 0, Data[9+t], LiftingTimer.COUNT_DOWN, onComplete1);
    
    // Start dipping operation
    Serial2.print("t10.txt=\"Dip\"\xFF\xFF\xFF");
    dippingTarget = 200 * Data[6+t];
    
    myStepper.setMaxSpeed(200 * Data[5+t]);
    myStepper.setAcceleration(20000);
    myStepper.moveTo(dippingTarget);
    
    operationInProgress = true;
    currentState = DIPPING;
    stateStartTime = millis();
  } else {
    checkTrayOperation();
  }
}

int getSimulationOffset() {
  if (count == 0) return 2;  // First simulation
  if (count == 1) return 9;  // Second simulation
  if (count == 2) return 16; // Third simulation
  if (count == 3) return 23; // Fourth simulation
  return -1;
}

void handleDippingState() {
  // Check if motor has reached target
  if (myStepper.distanceToGo() == 0) {
    // Start the dipping timer
    DippingTimer.start();
    myStepper.setCurrentPosition(0);
    
    if (!DippingTimer.isCounterCompleted()) {
      // Wait for dipping timer to complete
      // No action needed - timer is running
    } else {
      // Dipping complete, start lifting
      LiftingTimer.pause();
      liftingTarget = -dippingTarget;
      
      myStepper.setMaxSpeed(200 * Data[8 + getSimulationOffset()]);
      myStepper.setAcceleration(20000);
      myStepper.moveTo(liftingTarget);
      
      Serial2.print("t10.txt=\"Lift\"\xFF\xFF\xFF");
      currentState = LIFTING;
      stateStartTime = millis();
    }
  }
}

void handleLiftingState() {
  // Check if motor has reached lifting target or hall sensor triggered
  if (myStepper.distanceToGo() == 0 || digitalRead(HALL_SENSOR) == HIGH) {
    // Start the lifting timer
    LiftingTimer.start();
    myStepper.setCurrentPosition(0);
    
    if (!LiftingTimer.isCounterCompleted()) {
      // Wait for lifting timer to complete
      // No action needed - timer is running
    } else {
      // Lifting complete, increment cycle count
      Cycles++;
      Serial2.print("n1.val=");
      Serial2.print(Cycles);
      Serial2.print("\xFF\xFF\xFF");
      
      DippingTimer.restart();
      LiftingTimer.restart();
      DippingTimer.pause();
      LiftingTimer.pause();
      
      // Check if we've completed all cycles for this position
      int t = getSimulationOffset();
      if (Cycles >= Data[10+t] && Data[10+t] >= 1) {
        // Move to tray waiting state
        setupTrayWaiting();
      } else {
        // Start another dipping cycle
        currentState = DIPPING;
        handleDippingState();
      }
    }
  }
}

void setupTrayWaiting() {
  int waitIndex = 0;
  
  if (count == 0) waitIndex = 1;
  else if (count == 1) waitIndex = 3;
  else if (count == 2) waitIndex = 5;
  else if (count == 3) waitIndex = 37;
  
  // Set up tray waiting time
  if (waitIndex > 0 && Data[waitIndex] > 0) {
    TrayTimer.setCounter(0, 0, Data[waitIndex], TrayTimer.COUNT_DOWN, onComplete2);
    TrayTimer.start();
    
    // Reset display values
    TimerSet();
    
    currentState = TRAY_WAITING;
    stateStartTime = millis();
    waitTime = Data[waitIndex] * 1000; // Convert to milliseconds
  } else {
    // Skip waiting and go directly to tray moving
    currentState = TRAY_MOVING;
    setupTrayMoving();
  }
}

void handleTrayWaitingState() {
  // Check if we've waited long enough
  if (TrayTimer.isCounterCompleted() || (millis() - stateStartTime) >= waitTime) {
    TrayTimer.pause();
    currentState = TRAY_MOVING;
    setupTrayMoving();
  }
}

void setupTrayMoving() {
  // Determine which tray movement to make based on count
  if (count < 3) {
    // Move to next position
    LiftHome();
    if (count == 2) {
      // For the third position, reverse motor tray
      reverse_motor_tray_move();
    } else {
      // For first and second positions, regular motor tray
      motor_tray_move();
    }
    
    count++;
    Cycles = 0;
    
    // Update display
    Serial2.print("n2.val=");
    Serial2.print(count + 1);
    Serial2.print("\xFF\xFF\xFF");
    
    // Go back to idle to check for next operation
    currentState = IDLE;
    operationInProgress = false;
  } else {
    // We've completed all positions in this cycle
    count = 0;
    
    if (total_cycle_count == Data[35]) {
      // All cycles complete
      total_cycle_count = 1;
      currentState = FINISHING;
    } else {
      // Move to next cycle
      total_cycle_count++;
      
      Serial2.print("n3.val=");
      Serial2.print(total_cycle_count);
      Serial2.print("\xFF\xFF\xFF");
      
      Serial2.print("t10.txt=\"NC\"\xFF\xFF\xFF");
      
      // Return to home position
      LiftHome();
      TrayHome();
      
      // Move to starting position
      myStepper_2.setMaxSpeed(200);
      myStepper_2.setAcceleration(200);
      myStepper_2.moveTo(450);
      
      currentState = IDLE;
      operationInProgress = false;
    }
  }
}

void handleTrayMovingState() {
  // Check if motors have completed their movements
  if (myStepper.distanceToGo() == 0 && myStepper_2.distanceToGo() == 0) {
    // Movement complete, go back to idle
    currentState = IDLE;
    operationInProgress = false;
    
    // Display updates would go here if needed
  }
}

void handleHomingState() {
  // Check if homing is complete
  static bool verticalHomeDone = false;
  static bool trayHomeDone = false;
  
  // Check vertical axis
  if (!verticalHomeDone) {
    if (digitalRead(HALL_SENSOR) == HIGH) {
      myStepper.setCurrentPosition(0);
      verticalHomeDone = true;
    } else {
      // Continue homing
      myStepper.setMaxSpeed(1000);
      myStepper.setAcceleration(2000);
      myStepper.moveTo(Homing);
      Homing--;
    }
  }
  
  // Check tray axis
  if (!trayHomeDone) {
    if (digitalRead(TRAY_HALL_SENSOR) == HIGH) {
      myStepper_2.setCurrentPosition(0);
      trayHomeDone = true;
    } else {
      // Continue homing
      myStepper_2.setMaxSpeed(250);
      myStepper_2.setAcceleration(125);
      myStepper_2.moveTo(TrayHoming);
      TrayHoming--;
    }
  }
  
  // If both are done, move to initial position
  if (verticalHomeDone && trayHomeDone) {
    // Move to initial positions
    myStepper_2.setMaxSpeed(200);
    myStepper_2.setAcceleration(200);
    myStepper_2.moveTo(450);
    
    myStepper.setMaxSpeed(1000);
    myStepper.setAcceleration(2000);
    myStepper.moveTo(5250);
    
    verticalHomeDone = false;
    trayHomeDone = false;
    
    // Update LEDs
    updateLEDs(0, 0, 255);
    
    // Return to idle state
    currentState = IDLE;
    operationInProgress = false;
  }
}

void handleFinishingState() {
  // Display completion message
  Serial2.print("n0.val=0\xFF\xFF\xFF");
  Serial2.print("n1.val=0\xFF\xFF\xFF");
  Serial2.print("n2.val=0\xFF\xFF\xFF");
  Serial2.print("n3.val=0\xFF\xFF\xFF");
  Serial2.print("n4.val=0\xFF\xFF\xFF");
  Serial2.print("t10.txt=\"Done\"\xFF\xFF\xFF");
  
  myStepper.setCurrentPosition(0);
  myStepper_2.setCurrentPosition(0);
  
  // Sound buzzer if enabled
  if (!buzzerEnabled) {
    unsigned long currentTime = millis();
    static unsigned long lastBuzzerTime = 0;
    
    if (currentTime - lastBuzzerTime > 1300) {
      lastBuzzerTime = currentTime;
      
      static bool buzzerOn = false;
      if (buzzerOn) {
        noTone(BUZZER_PIN);
        buzzerOn = false;
      } else {
        tone(BUZZER_PIN, 350);
        buzzerOn = true;
      }
      
      // Check for cancel command
      if (global_rc == 'k' || global_rc == 'o') {
        noTone(BUZZER_PIN);
        homefunction();
        currentState = IDLE;
        operationInProgress = false;
      }
    }
  }
}

void checkTrayOperation() {
  // This combines the previous TrayCheck logic
  if (count == 0) {
    if (Data[7] == 0 || (Data[14] != 0 || Data[21] != 0 || Data[28] != 0) || global_rc != 'g') {
      resetSystem();
      Next();
      LiftHome();
      motor_tray_move();
    }
  } else if (count == 1) {
    if (Data[21] != 0 || Data[28] != 0 || global_rc != 'g') {
      resetSystem();
      Next();
      LiftHome();
      motor_tray_move();
    }
  } else if (count == 2) {
    if (Data[28] != 0 || global_rc != 'g') {
      resetSystem();
      Next();
      LiftHome();
      reverse_motor_tray_move();
    }
  }
}

// Non-blocking home function
void homefunction() {
  updateLEDs(255, 0, 0);
  
  Homing = -1;
  TrayHoming = -1;
  
  currentState = HOMING;
  operationInProgress = true;
}

void updateLEDs(int r, int g, int b) {
  for (int i = 2; i >= 0; i--) {
    leds[i] = CRGB(r, g, b);
  }
  FastLED.show();
}

// Helper functions that remain largely unchanged
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial2.available() > 0) {
    rc = Serial2.read();
    delay(1); // Keep short delay to prevent issues with rapid serial reads
    
    if (rc == startMarker) {
      recvInProgress = true;
    }
    else if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        Serial.println(receivedChars);
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    
    // Handle immediate commands
    handleImmediateCommands(rc);
    global_rc = rc;
  }
}

void handleImmediateCommands(char rc) {
  if (rc == 'U') {
    Serial.println("Move Up");
    MoveUp();
  }
  else if (rc == 'D') {
    Serial.println("Move Down");
    MoveDown();
  }
  else if (rc == 'H') {
    Serial.println("Homing");
    homefunction();
  }
  else if (rc == 'S') {
    Serial.println("Stop");
    StopMotor();
  }
  else if (rc == 'R') {
    Serial.println("Sound");
    buzzerEnabled = !buzzerEnabled;
    if (!buzzerEnabled) {
      tone(BUZZER_PIN, 350);
      delay(1000);
      noTone(BUZZER_PIN);
    }
  }
  else if (rc == 'g') {
    operationInProgress = false;
    homefunction();
  }
}

void parseData() {
  char * strtokIndx; //  strtok()  index
  strtokIndx = strtok(tempChars,",");
  while (strtokIndx != NULL && valuesCount < 74){
    Data[valuesCount++] = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");
  }
  valuesCount = 0; // Reset the Count
  
  // Set data indicators
  Data[0]='X';
  Data[2]='Y';
  Data[4]='Z';
  Data[6]='A';
  Data[13]='B';
  Data[20]='C';
  Data[27]='D';
  Data[34]='m';
  Data[36]='h';
  Data[38]='t';
  Data[44]='e';
  Data[50]='p';
  Data[56]='y';
  Data[62]='s';
  Data[68]='w';
}

void showParsedData() {
  for(int e = 0; e < 74; e++){
    Serial.print(Data[e]);
    Serial.print(" ");
  }
  Serial.println();
}

// Timer display functions
void DippingTimeDisp() {
   Serial2.print("n0.val=");
   Serial2.print(DippingTimer.getCurrentSeconds());
   Serial2.print("\xFF\xFF\xFF");
}

void LiftingTimeDisp() {
   Serial2.print("n0.val=");
   Serial2.print(LiftingTimer.getCurrentSeconds());
   Serial2.print("\xFF\xFF\xFF");
}

void TrayTimeDisp() {
   Serial2.print("n4.val=");
   Serial2.print(TrayTimer.getCurrentSeconds());
   Serial2.print("\xFF\xFF\xFF");
}

// Empty callbacks
void onComplete() {}
void onComplete1() {}
void onComplete2() {}

// Stepper control functions
void LiftHome() {
  // Non-blocking approach
  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(2000);
  
  // First move up to find hall sensor
  while (digitalRead(HALL_SENSOR) == LOW) {
    myStepper.moveTo(Homing);
    Homing--;
    myStepper.run();
  }
  
  myStepper.setCurrentPosition(0);
}

void TrayHome() {
  myStepper_2.setMaxSpeed(250);
  myStepper_2.setAcceleration(125);
  
  while (digitalRead(TRAY_HALL_SENSOR) == LOW) {
    myStepper_2.moveTo(TrayHoming);
    TrayHoming--;
    myStepper_2.run();
  }
  
  myStepper_2.setCurrentPosition(0);
}

void MoveUp() {
  myStepper.setCurrentPosition(0);
  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(2000);
  myStepper.moveTo(-4000);
}

void MoveDown() {
  myStepper.setCurrentPosition(0);
  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(2000);
  myStepper.moveTo(4000);
}

void StopMotor() {
  myStepper.setCurrentPosition(0);
  myStepper.stop();
}

void StopMotor_2() {
  myStepper_2.setCurrentPosition(0);
  myStepper_2.stop();
}

void motor_tray_move() {
  myStepper_2.setCurrentPosition(0);
  myStepper_2.setMaxSpeed(200);
  myStepper_2.setAcceleration(200);
  myStepper_2.moveTo(725);
}

void reverse_motor_tray_move() {
  myStepper_2.setCurrentPosition(0);
  myStepper_2.setMaxSpeed(200);
  myStepper_2.setAcceleration(200);
  myStepper_2.moveTo(-725);
}

void TimerSet() {
  Serial2.print("n0.val=0\xFF\xFF\xFF");
  Serial2.print("n1.val=0\xFF\xFF\xFF");
  Serial2.print("n2.val=0\xFF\xFF\xFF");
}

void Next() {
  Serial2.print("page 4");
  Serial2.print("\xFF\xFF\xFF");
  Serial2.print("n4.val=0\xFF\xFF\xFF");
  Serial2.print("n3.val=");
  Serial2.print(total_cycle_count);
  Serial2.print("\xFF\xFF\xFF");
  Serial2.print("t10.txt=\"Next\"\xFF\xFF\xFF");
}