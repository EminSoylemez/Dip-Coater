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
bool flage=1;
bool flage2=1;
bool flage3=0;
bool flage4=1;
bool flage5=0;
bool flage6=0;
bool Donesim=1;
bool activelight=0;
int count=0;
char global_rc;
int total_cycle_count = 1;
int currentPosition = 0;

void setup() {
  pinMode(EN_1, OUTPUT);
  digitalWrite(EN_1, LOW);
  pinMode(EN_2, OUTPUT);
  digitalWrite(EN_2, LOW);
  pinMode(BUZZER_PIN,OUTPUT);
  //FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  pinMode(HALL_SENSOR, INPUT);
  pinMode(TRAY_HALL_SENSOR, INPUT);
  homefunction();
  Serial.begin(9600);
  Serial2.begin(9600);
  //DippingTimer.setCounter(0, 0, 10, DippingTimer.COUNT_UP, onComplete); 
  //LiftingTimer.setCounter(0, 0, 10, LiftingTimer.COUNT_UP, onComplete1);
  DippingTimer.setInterval(DippingTimeDisp, 1000);   // For Displaying Time every 1sec
  LiftingTimer.setInterval(LiftingTimeDisp, 1000);
  TrayTimer.setInterval(TrayTimeDisp, 1000);
}

void loop() {
  DippingTimer.run();
  LiftingTimer.run();
  recvWithStartEndMarkers();
  if (newData == true){
    memcpy(tempChars, receivedChars, numChars * sizeof(char));
    parseData();
    showParsedData();
    flage=1;
    flage2=1;
    flage3=1;
    flage4=1;
    flage5=1;
    Donesim=1;
    Cycles=0;
    count=0;
    currentPosition = myStepper.currentPosition();
    newData = false;
  }
  
  CheckCycle(0);
  CheckCycle(6);
  CheckCycle(12);
  CheckCycle(18);
  CheckCycle(24);
  CheckCycle(30);

  if(Donesim==1){
    if(count==0){
      Serial2.print("n2.val=1\xFF\xFF\xFF");
      Sumulation(2);
    }
    else if(count==1){
      Serial2.print("n2.val=2\xFF\xFF\xFF");
      Sumulation(9);
    }
    else if(count==2){
      Serial2.print("n2.val=3\xFF\xFF\xFF");
      Sumulation(16);
    }
    else if(count==3){
      Serial2.print("n2.val=4\xFF\xFF\xFF");
      Sumulation(23);
    }
  }
  
  if(activelight==1){
    for (int i = 2; i >= 0; i--) {
      leds[i] = CRGB ( 0, 0, 255);
      FastLED.show();
      activelight=0;
    }
  }
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial2.available() > 0) {
    rc = Serial2.read();
    delay(1);
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
    if (rc=='U'){
      Serial.println("Move Up");
      MoveUp();
    }
    else if (rc=='D'){
      Serial.println("Move Down");
      MoveDown();
    }
    else if (rc=='H'){
      Serial.println("Homing");
      homefunction();
    }
    else if (rc=='S'){
      Serial.println("Stop");
      StopMotor();           
    }
    else if (rc=='R'){
      Serial.println("Sound");
      if(flage6==0){
        flage6=1;
      }
      else{
        tone(BUZZER_PIN, 350); // Send 1KHz sound signal...
        delay(1000);        // ...for 1 sec
        noTone(BUZZER_PIN);     // Stop sound...
        flage6=0;
      }
    }
    else if (rc=='Q'){
      Serial.println("LED");
    }
    else if(rc=='g'){
      Donesim=0;
      Data[74]={0.0};
      homefunction();
    }
    global_rc=rc;
  }
}
void parseData() {      // split the data into  parts and convert it to Float
  char * strtokIndx; //  strtok()  index
  strtokIndx = strtok(tempChars,",");
  while (strtokIndx != NULL && valuesCount < 74){
    Data[valuesCount++] = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");
  }
  valuesCount = 0; // Reset the Count
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

void showParsedData() {   // For Deubgging
  for(int e = 0;e<74;e++){
    Serial.print(Data[e]);
    Serial.print(" ");
  }
  Serial.println();
}

void DippingTimeDisp(){ // Displaying Dipping Time
   Serial2.print("n0.val=");
   Serial2.print(DippingTimer.getCurrentSeconds());
   Serial2.print("\xFF\xFF\xFF");
}
void LiftingTimeDisp(){ // Displaying Lifting Time
   Serial2.print("n0.val=");
   Serial2.print(LiftingTimer.getCurrentSeconds());
   Serial2.print("\xFF\xFF\xFF");
}
void TrayTimeDisp(){ // Displaying Lifting Time
   Serial2.print("n4.val=");
   Serial2.print(TrayTimer.getCurrentSeconds());
   Serial2.print("\xFF\xFF\xFF");
}

void onComplete() { // For Deubgging 
}

void onComplete1() { // For Deubgging 
}
void onComplete2() { // For Deubgging 
}
void onComplete3() { // For Deubgging 
}


void homefunction() {
  for (int i = 2; i >= 0; i--) {
    leds[i] = CRGB ( 255, 0, 0);
    FastLED.show();
  }

  while (digitalRead(HALL_SENSOR) == LOW) {
    myStepper.setMaxSpeed(1000);
    myStepper.setAcceleration(2000);
    myStepper.moveTo(Homing); //CCW
    Homing--;
    myStepper.run();
  }
  myStepper.setCurrentPosition(0); //reset postion
  TrayHome();

  myStepper_2.setMaxSpeed(200);
  myStepper_2.setAcceleration(200);
  myStepper_2.moveTo(450); //CW 1mm increment
  myStepper_2.runToPosition();
  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(2000);
  myStepper.moveTo(5250); //CW 1mm increment
  myStepper.runToPosition();
  
  myStepper.setCurrentPosition(0); //reset postion
  myStepper_2.setCurrentPosition(0); //reset postion
  activelight=1;
}

void LiftHome(){
  while (digitalRead(HALL_SENSOR) == LOW) {
    myStepper.setMaxSpeed(1000);
    myStepper.setAcceleration(2000);
    myStepper.moveTo(Homing); //CCW
    Homing--;
    myStepper.run();
  }
  Homing=0;
  while (digitalRead(HALL_SENSOR) == HIGH) {
    myStepper.setMaxSpeed(1000);
    myStepper.setAcceleration(2000);
    myStepper.moveTo(Homing); //CCW
    Homing++;
    myStepper.run();
  }
}

void TrayHome(){
  while(digitalRead(TRAY_HALL_SENSOR) == LOW){ //if else atılacak pinleri ayırmak için !
    myStepper_2.setMaxSpeed(250);
    myStepper_2.setAcceleration(125);
    myStepper_2.moveTo(TrayHoming); //CW 1mm increment
    TrayHoming--;
    myStepper_2.run();
  }
  myStepper_2.setCurrentPosition(0); //reset postion
}
void MoveUp(){
    myStepper.setCurrentPosition(0); //reset postion 
    myStepper.setMaxSpeed(1000);
    myStepper.setAcceleration(2000);
    myStepper.moveTo(-4000); //CCW 10mm increment 
    while( myStepper.distanceToGo() !=0 && digitalRead(HALL_SENSOR) == 0 && Serial2.available()==0 ){
     myStepper.run();
  } 
   myStepper.setCurrentPosition(0); //reset postion 
}
void MoveDown(){
    myStepper.setCurrentPosition(0); //reset postion 
    myStepper.setMaxSpeed(1000);
    myStepper.setAcceleration(2000);
    myStepper.moveTo(4000);
    while( myStepper.distanceToGo() !=0 && Serial2.available()==0){
      myStepper.run();
    }
   myStepper.setCurrentPosition(0); //reset postion 
}
void StopMotor(){
   myStepper.setCurrentPosition(0); //reset postion 
   myStepper.stop();
}
void StopMotor_2(){
   myStepper_2.setCurrentPosition(0); //reset postion 
   myStepper_2.stop();
}
void motor_tray_move(){
  myStepper_2.setCurrentPosition(0);
  myStepper_2.setMaxSpeed(200);
  myStepper_2.setAcceleration(200);
  myStepper_2.moveTo(725); //CCW 10mm increment
  while( myStepper_2.distanceToGo() !=0 && digitalRead(TRAY_HALL_SENSOR) == 0){ //Hall Sensör kontrol edilecek, diğer motor en tepeye ulaştığında dönmesi lazım
    myStepper_2.run();
  }
  myStepper_2.setCurrentPosition(0); //reset postion 
}

void reverse_motor_tray_move(){
  myStepper_2.setCurrentPosition(0);
  myStepper_2.setMaxSpeed(200);
  myStepper_2.setAcceleration(200);
  myStepper_2.moveTo(-725); //CW 10mm increment
  while( myStepper_2.distanceToGo() !=0 && digitalRead(TRAY_HALL_SENSOR) == 0){ //Hall Sensör kontrol edilecek, diğer motor en tepeye ulaştığında dönmesi lazım
    myStepper_2.run();
  }
  myStepper_2.setCurrentPosition(0); //reset postion
}

void reset() {
  flage = 1;
  flage2 = 1;
  flage3 = 1;
  flage4 = 0;
  flage5 = 1;
  Cycles = 0;
  Dipped = 1;
  Withdraw = 1;
  Donesim = 1;
  // Reset Nextion screen values
  DippingTimer.setInterval(DippingTimeDisp, 1000);   // For Displaying Time every 1sec
  LiftingTimer.setInterval(LiftingTimeDisp, 1000);
  TrayTimer.setInterval(TrayTimeDisp, 1000);
}

void Done(){
  Donesim=0;
  Serial2.print("n0.val=0\xFF\xFF\xFF");
  Serial2.print("n1.val=0\xFF\xFF\xFF");
  Serial2.print("n2.val=0\xFF\xFF\xFF");
  Serial2.print("n3.val=0\xFF\xFF\xFF");
  Serial2.print("n4.val=0\xFF\xFF\xFF");
  Serial2.print("t10.txt=");
  Serial2.print("\"");
  Serial2.print("Done");
  Serial2.print("\"");
  Serial2.print("\xFF\xFF\xFF");
  /*Homing=0;
  TrayHoming=0;
  while (digitalRead(HALL_SENSOR) == LOW) {
    myStepper.setMaxSpeed(500);
    myStepper.setAcceleration(1000); 
    myStepper.moveTo(Homing); //CCW
    Homing--;
    myStepper.run();
  }
  while (digitalRead(TRAY_HALL_SENSOR) == LOW) {
    myStepper_2.setMaxSpeed(200);
    myStepper_2.setAcceleration(200);
    myStepper_2.moveTo(TrayHoming); //CW 1mm increment
    TrayHoming--;
    myStepper_2.run();
  }
  myStepper.setCurrentPosition(0); //reset postion
  myStepper.setMaxSpeed(1000);
  myStepper.setAcceleration(2000);
  myStepper.moveTo(5250);
  while( myStepper.distanceToGo() !=0 && Serial2.available()==0){
    myStepper.run();
  }

  myStepper_2.setCurrentPosition(0); //reset postion
  myStepper_2.setMaxSpeed(200);
  myStepper_2.setAcceleration(200);
  myStepper_2.moveTo(375);
  while( myStepper_2.distanceToGo() !=0 && Serial2.available()==0){
    myStepper_2.run();
  }
  */
  myStepper.setCurrentPosition(0); //reset postion
  myStepper_2.setCurrentPosition(0); //reset postion
  if(flage6==0){
    while(true){
      tone(BUZZER_PIN, 350); // Send 1KHz sound signal...
      delay(1000);        // ...for 1 sec
      noTone(BUZZER_PIN);     // Stop sound...
      delay(300);
      global_rc = Serial2.read();
      if(global_rc=='k' || global_rc=='o'){
        homefunction();
        break;
      }
    }
    noTone(BUZZER_PIN);
  }
}

void Next(){
  Serial2.print("page 4"); // Upload Simulation Screen In Nextion Display 
  Serial2. print("\xFF\xFF\xFF");
  Serial2.print("n4.val=0\xFF\xFF\xFF");
  Serial2.print("n3.val=");
  Serial2.print(total_cycle_count);
  Serial2.print("\xFF\xFF\xFF");
  Serial2.print("t10.txt=");
  Serial2.print("\"");
  Serial2.print("Next");
  Serial2.print("\"");
  Serial2.print("\xFF\xFF\xFF");
}

void TimerSet(){
  Serial2.print("n0.val=0\xFF\xFF\xFF");
  Serial2.print("n1.val=0\xFF\xFF\xFF");
  Serial2.print("n2.val=0\xFF\xFF\xFF");
}

void TrayCheck(){
  if(count == 0){
    if(Data[7] == 0 || (Data[14]!=0 || Data[21]!=0 || Data[28]!=0) || global_rc!='g'){
      Serial.println("Hoşgeldin Ebubekir Sıddık Bebek");
      reset();
      Next();
      LiftHome();
      motor_tray_move();
    }
    else{
      //Done();
    }
  }
  else if(count == 1){
    if(Data[21]!=0 || Data[28]!=0 || global_rc!='g'){
      Serial.println("Şemseddin");
      reset();
      Next();
      LiftHome();
      motor_tray_move();
    }
    else{
      //Done();
    }
  }

  else if(count == 2){
    if(Data[28]!=0 || global_rc!='g'){
      Serial.println("Fatih Sultan Mehmet");
      reset();
      Next();
      LiftHome();
      reverse_motor_tray_move();
    }
    else{
      //Done();
    }
  }
}

void CheckCycle(int a){
  if(Data[39+a]==total_cycle_count && total_cycle_count!=Data[35]){
    if(Data[40+a]==0 && count==0){
      count=1;
      Serial.println("BOM-1");
      LiftHome();
      motor_tray_move();
    }
    else if(Data[41+a]==0 && count==1){
      count=2;
      Serial.println("BOM-2");
      LiftHome();
      motor_tray_move();
    }
    else if(Data[42+a]==0 && count==2){
      count=3;
      Serial.println("BOM-3");
      LiftHome();
      reverse_motor_tray_move();
    }
    else if(Data[43+a]==0 && count==3){
      count=0;
      total_cycle_count++;
      Serial.println("BOM-4");
      TrayTimer.setCounter(0, 0, Data[37], TrayTimer.COUNT_DOWN, onComplete2);
      TrayTimer.start();
      TimerSet();
      for(int i = 0;i < 5*Data[37];i+=1){
        TrayTimer.run();
        TrayTimer.setInterval(TrayTimeDisp, 1000);
        delay(200);
      }
      Serial2.print("n0.val=0\xFF\xFF\xFF");
      Serial2.print("n1.val=0\xFF\xFF\xFF");
      Serial2.print("n2.val=0\xFF\xFF\xFF");
      Serial2.print("n3.val=");
      Serial2.print(total_cycle_count);
      Serial2.print("\xFF\xFF\xFF");
      Serial2.print("n4.val=0\xFF\xFF\xFF");
      Serial2.print("t10.txt=");
      Serial2.print("\"");
      Serial2.print("NC");
      Serial2.print("\"");
      Serial2.print("\xFF\xFF\xFF");
      LiftHome();
      while (digitalRead(TRAY_HALL_SENSOR) == LOW) {
        myStepper_2.setMaxSpeed(200);
        myStepper_2.setAcceleration(200);
        myStepper_2.moveTo(TrayHoming); //CW 1mm increment
        TrayHoming--;
        myStepper_2.run();
      }
      myStepper_2.setCurrentPosition(0); //reset postion
      myStepper_2.setMaxSpeed(200);
      myStepper_2.setAcceleration(200);
      myStepper_2.moveTo(400);
      while( myStepper_2.distanceToGo() !=0 && Serial2.available()==0){
        myStepper_2.run();
      }
      reset();
    }
  }
}

void Sumulation(int t){

  Serial2.print("n3.val=");
  Serial2.print(total_cycle_count);
  Serial2.print("\xFF\xFF\xFF");

  if(Data[5+t]>0 && Data[6+t]>0 && Data[7+t]>0 && Data[8+t]>0 && Data[9+t]>0 && Data[5+t]<=20 && Data[8+t]<=20 && global_rc!='g'){
    if(Cycles<Data[10+t] && Data[10+t]>=1.0 ){
        if(flage==1){ // Upload the Simualtion Page for one time only 
        Serial2.print("page 4"); // Upload Simulation Screen In Nextion Display 
        Serial2. print("\xFF\xFF\xFF");
        reset();
        for (int i = 0; i <= 2; i++) {
        leds[i] = CRGB ( 0, 255, 0);
        FastLED.show();
        }
        flage=0;
        }
        if(flage2==1){ // Setting the Time for the counter 
          DippingTimer.setCounter(0, 0, Data[7+t], DippingTimer.COUNT_DOWN, onComplete); 
          LiftingTimer.setCounter(0, 0, Data[9+t], LiftingTimer.COUNT_DOWN, onComplete1);
          flage2=0;
        }
        
    //////////////////// Dipping Process //////////////////////
      if(!DippingTimer.isCounterCompleted()){
        Serial2.print("t10.txt=");
        Serial2.print("\"");
        Serial2.print("Dip");
        Serial2.print("\"");
        Serial2.print("\xFF\xFF\xFF");
      // myStepper.setCurrentPosition(0); //reset postion 
        if(Dipped==1){
          myStepper.setMaxSpeed(200*Data[5+t]);
          myStepper.setAcceleration(20000);
          myStepper.moveTo(200*Data[6+t]);
          while(myStepper.distanceToGo() !=0 ){
            myStepper.run();
            if(global_rc=='g'){
              myStepper.distanceToGo() == 0;
            }
          }
        }
        if(myStepper.distanceToGo() == 0 && Dipped==1){
          DippingTimer.start(); //Start The Dipping Time
          myStepper.setCurrentPosition(0); //reset postion 
          Dipped=0;
        }
      }
      
    //////////////////// Withdrawal Process //////////////////////
      if( DippingTimer.isCounterCompleted() && !LiftingTimer.isCounterCompleted() ){ 
        Serial2.print("t10.txt=");
        Serial2.print("\"");
        Serial2.print("Lift");
        Serial2.print("\"");
        Serial2.print("\xFF\xFF\xFF");
        //myStepper.setCurrentPosition(0); //reset postion 
        if(Withdraw==1){
          myStepper.setMaxSpeed(200*Data[8+t]);
          myStepper.setAcceleration(20000); 
          myStepper.moveTo(-200*Data[6+t]); 
          while(myStepper.distanceToGo() !=0 &&  digitalRead(HALL_SENSOR) == 0 ){
            myStepper.run();
            if(global_rc=='g'){
              myStepper.distanceToGo() == 0;
            }
          }
        }
        if(myStepper.distanceToGo() == 0 &&  Withdraw==1){
          LiftingTimer.start(); //Start The Lifting Time
          myStepper.setCurrentPosition(0); //reset postion 
          Withdraw=0; 
        }
      }
    //////////////////// Reset Process ///////////////////////////
      if(DippingTimer.isCounterCompleted() && LiftingTimer.isCounterCompleted()){
            Cycles++;
            Serial2.print("n1.val=");
            Serial2.print(Cycles);
            Serial2.print("\xFF\xFF\xFF");
            Dipped=1;
            Withdraw=1;
            DippingTimer.restart();
            LiftingTimer.restart();
            DippingTimer.pause();
            LiftingTimer.pause();
      }
    } else  if(Cycles>=Data[10+t] && Data[10+t]>=1){
        if(count==0){
          TrayTimer.setCounter(0, 0, Data[1], TrayTimer.COUNT_DOWN, onComplete2);
          TrayTimer.start();
          TimerSet();
          for(int i = 0;i < 5*Data[1];i+=1){
            TrayTimer.run();
            TrayTimer.setInterval(TrayTimeDisp, 1000);
            delay(200);
          }
          TrayCheck();
        }
        else if(count==1){
          TrayTimer.setCounter(0, 0, Data[3], TrayTimer.COUNT_DOWN, onComplete2);
          TrayTimer.start();
          TimerSet();
          for(int i = 0;i < 5*Data[3];i+=1){
            TrayTimer.run();
            TrayTimer.setInterval(TrayTimeDisp, 1000);
            delay(200);
          }
          TrayCheck();
        }
        else if(count==2){
          TrayTimer.setCounter(0, 0, Data[5], TrayTimer.COUNT_DOWN, onComplete2);
          TrayTimer.start();
          TimerSet();
          for(int i = 0;i < 5*Data[5];i+=1){
            TrayTimer.run();
            TrayTimer.setInterval(TrayTimeDisp, 1000);
            delay(200);
          }
          TrayCheck();
        }
        else if(count==3){
          if(total_cycle_count==Data[35]){
            total_cycle_count=1;
            flage3=0;
            Done();
          }
          else{
            total_cycle_count++;
            Serial.println("Yukarı");
            Serial.println(total_cycle_count);
            TrayTimer.setCounter(0, 0, Data[37], TrayTimer.COUNT_DOWN, onComplete2);
            TrayTimer.start();
            TimerSet();
            for(int i = 0;i < 5*Data[37];i+=1){
              TrayTimer.run();
              TrayTimer.setInterval(TrayTimeDisp, 1000);
              delay(200);
            }
            Serial2.print("n0.val=0\xFF\xFF\xFF");
            Serial2.print("n1.val=0\xFF\xFF\xFF");
            Serial2.print("n2.val=0\xFF\xFF\xFF");
            Serial2.print("n3.val=");
            Serial2.print(total_cycle_count);
            Serial2.print("\xFF\xFF\xFF");
            Serial2.print("n4.val=0\xFF\xFF\xFF");
            Serial2.print("t10.txt=");
            Serial2.print("\"");
            Serial2.print("NC");
            Serial2.print("\"");
            Serial2.print("\xFF\xFF\xFF");
            LiftHome();
            while (digitalRead(TRAY_HALL_SENSOR) == LOW) {
              myStepper_2.setMaxSpeed(200);
              myStepper_2.setAcceleration(200);
              myStepper_2.moveTo(TrayHoming); //CW 1mm increment
              TrayHoming--;
              myStepper_2.run();
            }
            myStepper_2.setCurrentPosition(0); //reset postion
            myStepper_2.setMaxSpeed(200);
            myStepper_2.setAcceleration(200);
            myStepper_2.moveTo(450);
            while( myStepper_2.distanceToGo() !=0 && Serial2.available()==0){
              myStepper_2.run();
            }
            count=-1;
            reset();
          }
        }
        Serial.println("Sultan Selim geliyor!");
        count++;
    }
  }
  else if(Data[5+t]<=0 || Data[6+t]<=0 || Data[7+t]<=0 || Data[8+t]<=0 || Data[9+t]<=0 || Data[5+t]>20 || Data[8+t]>20 || global_rc=='g'){
    Serial.println(count);
    if(count==3 && flage3==1){
      if(total_cycle_count==Data[35]){
        Serial.println("69");
        total_cycle_count=1;
        flage3=0;
        Done();
      }
      else{
        Serial.println("77");
        total_cycle_count++;
        Serial.println("Aşağı");
        Serial.println(total_cycle_count);
        TrayTimer.setCounter(0, 0, Data[37], TrayTimer.COUNT_DOWN, onComplete2);
        TrayTimer.start();
        TimerSet();
        for(int i = 0;i < 5*Data[37];i+=1){
          TrayTimer.run();
          TrayTimer.setInterval(TrayTimeDisp, 1000);
          delay(200);
        }
        Serial2.print("n0.val=0\xFF\xFF\xFF");
        Serial2.print("n1.val=0\xFF\xFF\xFF");
        Serial2.print("n2.val=0\xFF\xFF\xFF");
        Serial2.print("n3.val=");
        Serial2.print(total_cycle_count);
        Serial2.print("\xFF\xFF\xFF");
        Serial2.print("n4.val=0\xFF\xFF\xFF");
        Serial2.print("t10.txt=");
        Serial2.print("\"");
        Serial2.print("NC");
        Serial2.print("\"");
        Serial2.print("\xFF\xFF\xFF");
        LiftHome();
        while (digitalRead(TRAY_HALL_SENSOR) == LOW) {
          myStepper_2.setMaxSpeed(200);
          myStepper_2.setAcceleration(200);
          myStepper_2.moveTo(TrayHoming); //CW 1mm increment
          TrayHoming--;
          myStepper_2.run();
        }
        myStepper_2.setCurrentPosition(0); //reset postion
        myStepper_2.setMaxSpeed(200);
        myStepper_2.setAcceleration(200);
        myStepper_2.moveTo(450);
        while(myStepper_2.distanceToGo() !=0 && Serial2.available()==0){
          myStepper_2.run();
        }
        count=-1;
        reset();
      }
    }
    if(flage5==1){
      Serial.println("Şakşuka");
      TrayCheck();
    }
    count++;
    Serial.println("Sultan Selim gelemedi!");
  }
} //MS