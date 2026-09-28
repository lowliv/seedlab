//Assignment 2 question 2a motor control
//Set pin numbers

#include <EnableInterrupt.h>
const int ENABLE = 4;
const int RIGHT = 8;
const int LEFT = 7;
const int PWM_RIGHT = 10;
const int PWM_LEFT = 9;

const int clkPinR = 2; // Right motor encoder clk
const int dtPinR = 3; // Right motor encoder dt

const int clkPinL = 5; // Left motor encoder clk
const int dtPinL = 6; // Left motor encoder dt

//set ISR variables
volatile long posR = 0; // Right raw encoder counts
volatile long posL = 0; // Left raw encoder counts

volatile int lastAR, lastBR, lastAL, lastBL; // Previous encoder variables for Isr

//Constants used for controls and calculations
const float batteryV = 7.6;
const float KpVEL = 0.5;
const float KiVEL = 2;
const float KpPOS = 0.5;
const float KiPOS = 2;

//Timing variables
const unsigned long Ts_ms = 10;
const float Ts = Ts_ms / 1000.0;
const float run_time = 3.0;
unsigned long start_time_ms, last_time_ms;
float prevPosRadR = 0;
float prevPosRadL = 0;

float integralR = 0;
float integralL = 0;
float integralErrorR = 0;
float integralErrorL = 0;

//Encoder ISR that is used to detect and count direction of motor encoder when turned.
void encoderIsrR() { // Encoder interrupt code right
  int thisAR = digitalRead(clkPinR);
  int thisBR = digitalRead(dtPinR);
  if (lastAR != thisAR || lastBR != thisBR) {
    if ((lastAR == 0 && lastBR == 0) && (thisAR == 0 && thisBR == 1)) ++posR;
    if ((lastAR == 0 && lastBR == 0) && (thisAR == 1 && thisBR == 0)) --posR;
    if ((lastAR == 0 && lastBR == 1) && (thisAR == 0 && thisBR == 0)) --posR;
    if ((lastAR == 0 && lastBR == 1) && (thisAR == 1 && thisBR == 1)) ++posR;
    if ((lastAR == 1 && lastBR == 0) && (thisAR == 0 && thisBR == 0)) ++posR;
    if ((lastAR == 1 && lastBR == 0) && (thisAR == 1 && thisBR == 1)) --posR;
    if ((lastAR == 1 && lastBR == 1) && (thisAR == 0 && thisBR == 1)) --posR;
    if ((lastAR == 1 && lastBR == 1) && (thisAR == 1 && thisBR == 0)) ++posR;
  }
  lastAR = thisAR;
  lastBR = thisBR;
}

//Encoder interrupt code left
void encoderIsrL() {
  int thisAL = digitalRead(clkPinL);
  int thisBL = digitalRead(dtPinL);
  if (lastAL != thisAL || lastBL != thisBL) {
    if ((lastAL == 0 && lastBL == 0) && (thisAL == 0 && thisBL == 1)) ++posL;
    if ((lastAL == 0 && lastBL == 0) && (thisAL == 1 && thisBL == 0)) --posL;
    if ((lastAL == 0 && lastBL == 1) && (thisAL == 0 && thisBL == 0)) --posL;
    if ((lastAL == 0 && lastBL == 1) && (thisAL == 1 && thisBL == 1)) ++posL;
    if ((lastAL == 1 && lastBL == 0) && (thisAL == 0 && thisBL == 0)) ++posL;
    if ((lastAL == 1 && lastBL == 0) && (thisAL == 1 && thisBL == 1)) --posL;
    if ((lastAL == 1 && lastBL == 1) && (thisAL == 0 && thisBL == 1)) --posL;
    if ((lastAL == 1 && lastBL == 1) && (thisAL == 1 && thisBL == 0)) ++posL;
  }
  lastAL = thisAL;
  lastBL = thisBL;
}

//Function to drive the motor that is called in loop after voltage is set.
//Handles -/+ voltages for forward and backwards turning.
float velDrive(int pin, float targetVel, float vel, float integral) {
  float error = targetVel - vel;
  integral += error * Ts;
  float voltage = KpVEL * error + KiVEL * integral;
  
  //make sure voltage is within range and drives motor
  voltage = constrain(voltage, -batteryV, batteryV);
  
  if (voltage >= 0) {
    digitalWrite(RIGHT, HIGH);
    digitalWrite(LEFT, LOW);
  } else {
    digitalWrite(RIGHT, LOW);
    digitalWrite(LEFT, HIGH);
  }
  int pwmLevel = (int)(255.0 * fabs(voltage) / batteryV);
  analogWrite(pin, pwmLevel);
  return integral;
}

float posDrive(int pwmPin, float targetPos, float pos, float vel, float integralError) {
  float posError = targetPos - pos;
  integralError = integralError + posError*((float)Ts_ms /1000);
  float targetVel = KpPOS * posError + KiPOS * integralError;
  float error = targetVel - vel;
  float voltage = KpPOS*error;

  //make sure voltage is within range and drives motor
  voltage = constrain(voltage, -batteryV, batteryV);
  
  if (voltage >= 0) {
    digitalWrite(RIGHT, HIGH);
    digitalWrite(LEFT, LOW);
  } else {
    digitalWrite(RIGHT, LOW);
    digitalWrite(LEFT, HIGH);
  }
  int pwmLevel = (int)(255.0 * fabs(voltage) / batteryV);
  analogWrite(pwmPin, pwmLevel);
  return integralError;
}

//Setup
void setup() {

  //Delay so there is time to run commands to start reading printed info
  delay(3000);
  Serial.begin(115200);
  
  //enable pins
  pinMode(ENABLE, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  pinMode(LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(PWM_LEFT, OUTPUT);
  digitalWrite(ENABLE, HIGH);

  pinMode(clkPinR, INPUT_PULLUP);
  pinMode(dtPinR, INPUT_PULLUP);
  pinMode(clkPinL, INPUT_PULLUP);
  pinMode(dtPinL, INPUT_PULLUP);

  // Set encoder starting states
  lastAR = digitalRead(clkPinR);
  lastBR = digitalRead(dtPinR);
  lastAL = digitalRead(clkPinL);
  lastBL = digitalRead(dtPinL);
  
  //set interupts
  enableInterrupt(clkPinR, encoderIsrR, CHANGE);
  enableInterrupt(dtPinR, encoderIsrR, CHANGE);
  enableInterrupt(clkPinL, encoderIsrL, CHANGE);
  enableInterrupt(dtPinL, encoderIsrL, CHANGE);
  
  //inialize start and last time to same value
  start_time_ms = last_time_ms = millis();
}

float targetPosR = 2 * PI;
float targetPosL = 2 * PI;
//Main loop
void loop() {
  //find the time in seconds since last loop
  float t = (last_time_ms - start_time_ms) / 1000.0;
  
  //turns counts into rad values and estimates velocity.
  float posRadR = 2 * PI * posR / 3200.0;
  float velR = (posRadR - prevPosRadR) / Ts;
  prevPosRadR = posRadR;


  //turns counts into rad values and estimates velocity.
  float posRadL = 2 * PI * posL / 3200.0;
  float velL = (posRadL - prevPosRadL) / Ts;
  prevPosRadL = posRadL;

  //float targetVelR = 1;
  //float targetVelL = 1;
  //integralR = velDrive(PWM_RIGHT, targetVelR, velR, integralR);
  //integralL = velDrive(PWM_LEFT, targetVelL, velL, integralL);

  float targetPosR = 2 * PI;
  float targetPosL = 2 * PI;
  integralErrorR = posDrive(PWM_RIGHT, targetPosR, posRadR, velR, integralErrorR);
  integralErrorL = posDrive(PWM_LEFT, targetPosL, posRadL, velL, integralErrorL);

  //prints data
  Serial.print(t);
  Serial.print(", ");
  Serial.print(posRadR);
  Serial.print(", ");
  Serial.println(posRadL);
  
  //waits set time to delay next loop cycle
  while (millis() < last_time_ms + Ts_ms) {}
  last_time_ms = millis();
}
