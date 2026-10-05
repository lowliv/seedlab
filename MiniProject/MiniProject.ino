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

const int clkPinR = 2; // Right motor encoder clk
const int dtPinR = 3; // Right motor encoder dt

const int clkPinL = 5; // Left motor encoder clk
const int dtPinL = 6; // Left motor encoder dt

// Set ISR variables
volatile long posR = 0; // Right raw encoder counts
volatile long posL = 0; // Left raw encoder counts
volatile int lastAR, lastBR, lastAL, lastBL; // Previous encoder variables for Isr

// Constants used for controls and calculations
const float batteryV = 7.6;
const float KpVEL = 1.4;
const float KiVEL = 2;
const float KpPOS = 1.2;
const float KiPOS = 0;

// Timing variables
const unsigned long Ts_ms = 10;
const float Ts = Ts_ms / 1000.0;
const float run_time = 3.0;
unsigned long start_time_ms, last_time_ms;

// Past positions
float prevPosRadR = 0;
float prevPosRadL = 0;

// Integral values
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

// Function to drive and control the motor based on target velocity
void velDrive(int pwmPin, int dirPin, float velTarget, float vel, float &velIntegral) {

  float velError = velTarget - vel;
  velIntegral += velError * Ts;
  float voltage = KpVEL * velError + KiVEL * velIntegral;
  
  // Make sure voltage is within range and drives motor
  voltage = constrain(voltage, -batteryV, batteryV);
  
  // Set direction if driving right
  if (voltage >= 0 && dirPin == RIGHT) {
    digitalWrite(RIGHT, HIGH);
  } else if (dirPin == RIGHT) {
    digitalWrite(RIGHT, LOW);
  }
  // Set direction if driving left
  if (voltage >= 0 && dirPin == LEFT) {
    digitalWrite(LEFT, LOW);
  } else if (dirPin == LEFT) {
    digitalWrite(LEFT, HIGH);
  }

  int pwmLevel = (int)(255.0 * fabs(voltage) / batteryV);
  analogWrite(pwmPin, pwmLevel);
}

// Function to drive and control the motor based on target position
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

void setup() {
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);

  // Delay so there is time to run commands to start reading printed info
  // delay(3000);
  Serial.begin(115200);
  
  // Enable pins
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

void loop() {
  // Find the time in seconds since last loop
  float t = (last_time_ms - start_time_ms) / 1000.0;
  
  // Turns counts into rad values and estimates velocity.
  float posRadR = -2 * PI * posR / 3200.0;
  float velR = (posRadR - prevPosRadR) / Ts;
  prevPosRadR = posRadR;


  // Turns counts into rad values and estimates velocity.
  float posRadL = 2 * PI * posL / 3200.0;
  float velL = (posRadL - prevPosRadL) / Ts;
  prevPosRadL = posRadL;

  //float velTargetR = 1;
  //float velTargetL = 1;
  //velDrive(PWM_RIGHT, RIGHT, velTargetR, velR, velIntegralR);
  //velDrive(PWM_LEFT, LEFT, velTargetL, velL, velIntegralL);
  float posTargetR;
  float posTargetL;
  
  if (cmdA) {
    posTargetR = PI;
  } else {
    posTargetR = 0;
  }
  if (cmdB) {
    posTargetL = PI;
  } else {
    posTargetL = 0;
  }


  posDrive(PWM_RIGHT, RIGHT, posTargetR, posRadR, velR, posIntegralR, velIntegralR);
  posDrive(PWM_LEFT, LEFT, posTargetL, posRadL, velL, posIntegralL, velIntegralL);
  
  if (newCmd) {
    newCmd = false;
    Serial.print("CMD: ");
    Serial.print(cmdA);
    Serial.print(", ");
    Serial.println(cmdB);
  }

  // Prints data
  /*
  Serial.print(t);
  Serial.print(", ");
  Serial.print(posRadR);
  Serial.print(", ");
  Serial.println(posRadL);
  */

  //waits set time to delay next loop cycle
  if (millis() > last_time_ms + Ts_ms) {
    Serial.println("WARNING: Ts too fast to handle");
  }
  while (millis() < last_time_ms + Ts_ms) {}
  last_time_ms = millis();
}
