//Assignment 2 question 2a motor control
//Set pin numbers

#include <EnableInterrupt.h>
#include <Wire.h>
#define MY_ADDR 8


volatile uint8_t cmdA = 0;
volatile uint8_t cmdB = 0;
volatile bool newCmd = true;

const int ENABLE = 4;
const int RIGHT = 8;
const int LEFT = 7;
const int PWM_RIGHT = 10;
const int PWM_LEFT = 9;

const int clkPinR = 2;
const int dtPinR = 3;

const int clkPinL = 5; 
const int dtPinL = 6;

//set ISR variables
volatile long posR = 0;
volatile long posL = 0;

volatile int lastAR, lastBR, lastAL, lastBL;

//Constants used for controls and calculations
const float batteryV = 7.6;
const float KpVEL = 0.5;
const float KiVEL = 2;
const float KpPOS = 0.5;
const float KiPOS = 0;

//Timing variables
const unsigned long Ts_ms = 10;
const float Ts = Ts_ms / 1000.0;
unsigned long start_time_ms, last_time_ms;
float prevPosRadR = 0;
float prevPosRadL = 0;

float posIntegralR = 0;
float posIntegralL = 0;
float velIntegralR = 0;
float velIntegralL = 0;

//Encoder ISR that is used to detect and count direction of motor encoder when turned.
void encoderIsrR() {
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
void velDrive(int pwmPin, int dirPin, float velTarget, float vel, float &velIntegral) {
  float velError = velTarget - vel;
  velIntegral += velError * Ts;

  // anti-windup: don't let the integral term alone exceed the available voltage
  float iLimit = batteryV / KiVEL;
  velIntegral = constrain(velIntegral, -iLimit, iLimit);

  float voltage = KpVEL * velError + KiVEL * velIntegral;
  
  //make sure voltage is within range and drives motor
  voltage = constrain(voltage, -batteryV, batteryV);
  
  // set direction ONLY for the motor being driven
  // (polarity is reversed so the motor pushes against the encoder error)
  digitalWrite(dirPin, voltage >= 0 ? LOW : HIGH);

  int pwmLevel = (int)(255.0 * fabs(voltage) / batteryV);
  analogWrite(pwmPin, pwmLevel);
}

void posDrive(int pwmPin, int dirPin, float posTarget, float pos, float vel, float &posIntegral, float &velIntegral) {
  float posError = posTarget - pos;
  posIntegral += posError * Ts;
  float velTarget = KpPOS * posError + KiPOS * posIntegral;

  velDrive(pwmPin, dirPin, velTarget, vel, velIntegral);
}

// I2C receive handler. Runs in interrupt context, so no Serial printing here.
// Reads every byte that arrived and uses the LAST TWO as the commands, so it
// works whether or not the Pi sends a leading register/offset byte.
void receive(int numBytes) {
  uint8_t prev = 0, last = 0;
  int count = 0;
  while (Wire.available()) {
    prev = last;
    last = Wire.read();
    count++;
  }
  if (count >= 2) {
    cmdA = prev;
    cmdB = last;
    newCmd = true;
  }
}

//Setup
void setup() {
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);

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

//Main loop
void loop() {
  //find the time in seconds since start
  float t = (last_time_ms - start_time_ms) / 1000.0;

  // copy the encoder counts atomically (they are changed inside ISRs)
  noInterrupts();
  long cntR = posR;
  long cntL = posL;
  interrupts();

  //turns counts into rad values and estimates velocity.
  float posRadR = 2 * PI * cntR / 3200.0;
  float velR = (posRadR - prevPosRadR) / Ts;
  prevPosRadR = posRadR;

  float posRadL = 2 * PI * cntL / 3200.0;
  float velL = (posRadL - prevPosRadL) / Ts;
  prevPosRadL = posRadL;

  // Set-points from the I2C commands: 0 -> 0 rad, 1 -> PI rad (180 deg)
  // cmdA = left wheel, cmdB = right wheel
  float posTargetL = (cmdA ? PI : 0.0);
  float posTargetR = (cmdB ? PI : 0.0);

  // Both wheels are controlled every cycle so they hold position and
  // reject disturbances (integral action keeps pulling them back).
  posDrive(PWM_RIGHT, RIGHT, posTargetR, posRadR, velR, posIntegralR, velIntegralR);
  posDrive(PWM_LEFT, LEFT, posTargetL, posRadL, velL, posIntegralL, velIntegralL);

  // print the received command once whenever a new one arrives
  if (newCmd) {
    newCmd = false;
    Serial.print("CMD: ");
    Serial.print(cmdA);
    Serial.print(", ");
    Serial.println(cmdB);
  }

  //prints data
  Serial.print(t);
  Serial.print(", ");
  Serial.print(posRadR);
  Serial.print(", ");
  Serial.println(posRadL);
  
  //waits set time to delay next loop cycle
  if (millis() > last_time_ms + Ts_ms) {
    Serial.println("WARNING: Ts too fast to handle");
  }
  while (millis() < last_time_ms + Ts_ms) {}
  last_time_ms = millis();
}//Assignment 2 question 2a motor control
//Set pin numbers

#include <EnableInterrupt.h>
#include <Wire.h>
#define MY_ADDR 8


volatile uint8_t cmdA = 0;   // Left wheel command  (0 = 0 deg, 1 = 180 deg)
volatile uint8_t cmdB = 0;   // Right wheel command (0 = 0 deg, 1 = 180 deg)
volatile bool newCmd = true;

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
const float KiPOS = 0;   // position integral removed: velocity-loop integrator already
                         // drives position error to zero, and two integrators oscillate

//Timing variables
const unsigned long Ts_ms = 10;
const float Ts = Ts_ms / 1000.0;
unsigned long start_time_ms, last_time_ms;
float prevPosRadR = 0;
float prevPosRadL = 0;

float posIntegralR = 0;
float posIntegralL = 0;
float velIntegralR = 0;
float velIntegralL = 0;

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
void velDrive(int pwmPin, int dirPin, float velTarget, float vel, float &velIntegral) {
  float velError = velTarget - vel;
  velIntegral += velError * Ts;

  // anti-windup: don't let the integral term alone exceed the available voltage
  float iLimit = batteryV / KiVEL;
  velIntegral = constrain(velIntegral, -iLimit, iLimit);

  float voltage = KpVEL * velError + KiVEL * velIntegral;
  
  //make sure voltage is within range and drives motor
  voltage = constrain(voltage, -batteryV, batteryV);
  
  // set direction ONLY for the motor being driven
  // (polarity is reversed so the motor pushes against the encoder error)
  digitalWrite(dirPin, voltage >= 0 ? LOW : HIGH);

  int pwmLevel = (int)(255.0 * fabs(voltage) / batteryV);
  analogWrite(pwmPin, pwmLevel);
}

void posDrive(int pwmPin, int dirPin, float posTarget, float pos, float vel, float &posIntegral, float &velIntegral) {
  float posError = posTarget - pos;
  posIntegral += posError * Ts;
  float velTarget = KpPOS * posError + KiPOS * posIntegral;

  velDrive(pwmPin, dirPin, velTarget, vel, velIntegral);
}

// I2C receive handler. Runs in interrupt context, so no Serial printing here.
// Reads every byte that arrived and uses the LAST TWO as the commands, so it
// works whether or not the Pi sends a leading register/offset byte.
void receive(int numBytes) {
  uint8_t prev = 0, last = 0;
  int count = 0;
  while (Wire.available()) {
    prev = last;
    last = Wire.read();
    count++;
  }
  if (count >= 2) {
    cmdA = prev;
    cmdB = last;
    newCmd = true;
  }
}

//Setup
void setup() {
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);

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

//Main loop
void loop() {
  //find the time in seconds since start
  float t = (last_time_ms - start_time_ms) / 1000.0;

  // copy the encoder counts atomically (they are changed inside ISRs)
  noInterrupts();
  long cntR = posR;
  long cntL = posL;
  interrupts();

  //turns counts into rad values and estimates velocity.
  float posRadR = 2 * PI * cntR / 3200.0;
  float velR = (posRadR - prevPosRadR) / Ts;
  prevPosRadR = posRadR;

  float posRadL = 2 * PI * cntL / 3200.0;
  float velL = (posRadL - prevPosRadL) / Ts;
  prevPosRadL = posRadL;

  // Set-points from the I2C commands: 0 -> 0 rad, 1 -> PI rad (180 deg)
  // cmdA = left wheel, cmdB = right wheel
  float posTargetL = (cmdA ? PI : 0.0);
  float posTargetR = (cmdB ? PI : 0.0);

  // Both wheels are controlled every cycle so they hold position and
  // reject disturbances (integral action keeps pulling them back).
  posDrive(PWM_RIGHT, RIGHT, posTargetR, posRadR, velR, posIntegralR, velIntegralR);
  posDrive(PWM_LEFT, LEFT, posTargetL, posRadL, velL, posIntegralL, velIntegralL);

  // print the received command once whenever a new one arrives
  if (newCmd) {
    newCmd = false;
    Serial.print("CMD: ");
    Serial.print(cmdA);
    Serial.print(", ");
    Serial.println(cmdB);
  }

  //prints data
  Serial.print(t);
  Serial.print(", ");
  Serial.print(posRadR);
  Serial.print(", ");
  Serial.println(posRadL);
  
  //waits set time to delay next loop cycle
  if (millis() > last_time_ms + Ts_ms) {
    Serial.println("WARNING: Ts too fast to handle");
  }
  while (millis() < last_time_ms + Ts_ms) {}
  last_time_ms = millis();
}
