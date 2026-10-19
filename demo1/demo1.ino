/* 

*/
#include <TimerOne_V2.h>
#include <EnableInterrupt.h>
#include <Wire.h>
#define MY_ADDR 8

volatile uint8_t cmdA = 0; // From PI
volatile uint8_t cmdB = 0; // From Pi
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

// Timing variables
const unsigned long Ts_ms = 3;
const float Ts = Ts_ms / 1000.0;
const float run_time = 8.0;
unsigned long start_time_ms, last_time_ms;

float batteryV = 3; // Battery Voltage

// Past positions
float prevPosRadR = 0;
float prevPosRadL = 0;

// Controller parameters
const float KpVELR = 1.358;
const float KiVELR = 2;
const float KpPOSR = 5.6;
const float KiPOSR = 0;
const float KpVELL = 1.4;
const float KiVELL = 2;
const float KpPOSL = 1.2;
const float KiPOSL = 0;

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
  float voltage;
  float velError = velTarget - vel;
  velIntegral += velError * Ts;
  if (dirPin == RIGHT) {
    voltage = KpVELR * velError + KiVELR * velIntegral;
  }
  if (dirPin == LEFT) {
    voltage = KpVELL * velError + KiVELL * velIntegral;
  }

  // Make sure voltage is within range and drives motor
  voltage = constrain(voltage, -batteryV, batteryV);
  
  // Set direction if driving right
  if (voltage >= 0 && dirPin == RIGHT) {
    digitalWrite(RIGHT, HIGH);
  } else if (dirPin == RIGHT) {
    digitalWrite(RIGHT, LOW);
  }
  // Set direction if driving left opposite because the left
  // is mirrored
  if (voltage >= 0 && dirPin == LEFT) {
    digitalWrite(LEFT, LOW);
  } else if (dirPin == LEFT) {
    digitalWrite(LEFT, HIGH);
  }

  //int pwmLevel = (int)(255.0 * fabs(voltage) / batteryV);
  //analogWrite(pwmPin, pwmLevel);
  Timer1.pwm(pwmPin, (1023 * fabs(voltage) / batteryV));
}

// Function to drive and control the motor based on target position
void posDrive(int pwmPin, int dirPin, float posTarget, float pos, float vel, float &posIntegral, float &velIntegral) {
  float velTarget;
  float posError = posTarget - pos;
  posIntegral += posError * Ts;
  if (dirPin == RIGHT) {
    velTarget = KpPOSR * posError + KiPOSR * posIntegral;
  }
  if (dirPin == LEFT) {
    velTarget = KpPOSL * posError + KiPOSL * posIntegral;
  }
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
  
  // Set interupts
  enableInterrupt(clkPinR, encoderIsrR, CHANGE);
  enableInterrupt(dtPinR, encoderIsrR, CHANGE);
  enableInterrupt(clkPinL, encoderIsrL, CHANGE);
  enableInterrupt(dtPinL, encoderIsrL, CHANGE);
  
  Timer1.frequency(25);
  // Inialize start and last time to same value
  start_time_ms = last_time_ms = millis();
}

void loop() {
  // Find the time in seconds since last loop
  float t = (last_time_ms - start_time_ms) / 1000.0;
  
  // Turns counts into rad values and estimates velocity.
  float posRadR = -2 * PI * posR / 3200.0;
  float velR = (posRadR - prevPosRadR) / Ts;
  prevPosRadR = posRadR;
  float posRadL = 2 * PI * posL / 3200.0;
  float velL = (posRadL - prevPosRadL) / Ts;
  prevPosRadL = posRadL;

  
  float velTargetR = 3;
  float velTargetL = 3;
  
  velDrive(PWM_RIGHT, RIGHT, velTargetR, velR, velIntegralR);
  velDrive(PWM_LEFT, LEFT, velTargetL, velL, velIntegralL);
  
  float posTargetR;
  float posTargetL;
  
  
  // Sets the target rotations in radians based on the bits recieved
  /*
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
  */
  //posTargetR = 6*PI;
  //posTargetL = 6*PI;

  // Call the position controllers for each wheel
  //posDrive(PWM_RIGHT, RIGHT, posTargetR, posRadR, velR, posIntegralR, velIntegralR);
  //posDrive(PWM_LEFT, LEFT, posTargetL, posRadL, velL, posIntegralL, velIntegralL);
  
  // Print statements for debugging
  /*
  if (newCmd) {
    newCmd = false;
    Serial.print("CMD: ");
    Serial.print(cmdA);
    Serial.print(", ");
    Serial.println(cmdB);
  }
  */

  // Prints statements for debugging
  
  Serial.print(t);
  Serial.print(", ");
  Serial.print(velL);
  Serial.print(", ");
  Serial.println(velR);


  // Waits set time to delay next loop cycle
  if (millis() > last_time_ms + Ts_ms) {
    Serial.println("WARNING: Ts too fast to handle");
  }
  while (millis() < last_time_ms + Ts_ms) {}
  last_time_ms = millis();
  while (last_time_ms > 5000) {
    Timer1.pwm(PWM_RIGHT, (0));
    Timer1.pwm(PWM_LEFT, (0));
  }
}
