/* 
Cammeron Macmillan and Wren Heinley Group #12
Demo 2 integration and controls

*/
#include <TimerOne_V2.h>
#include <EnableInterrupt.h>
#include <Wire.h>
#define MY_ADDR 8

//set variables to get data form PI
volatile uint8_t cmdA = 0;
volatile uint8_t cmdB = 0;
volatile bool newCmd = true;

//set pins
const int ENABLE = 4;
const int RIGHT = 7;
const int LEFT = 8;
const int PWM_RIGHT = 9;
const int PWM_LEFT = 10;

const int clkPinR = 5; // Right motor encoder clk
const int dtPinR = 6; // Right motor encoder dt

const int clkPinL = 2; // Left motor encoder clk
const int dtPinL = 3; // Left motor encoder dt

// Set ISR variables
volatile long posR = 0; // Right raw encoder counts
volatile long posL = 0; // Left raw encoder counts
volatile int lastAR, lastBR, lastAL, lastBL; // Previous encoder variables for Isr

// Timing variables
const unsigned long Ts_ms = 3;
const float Ts = Ts_ms / 1000.0;
const float run_time = 8.0;
unsigned long start_time_ms, last_time_ms;

//set values for turning
const float radPerIn = 0.341;
const float radPerDeg = 0.0416;
float batteryV = 7.6; // Battery Voltage

// Controller parameters
const float KpVELR = 1.4;
const float KiVELR = 2;
const float KpPOSR = 1;
const float KiPOSR = 0;

const float KpVELL = 1.4;
const float KiVELL = 2;
const float KpPOSL = 1;
const float KiPOSL = 0;

const float KpSYNC = 2.4;
//const float baseMaxVel = 4; 
//const float minMaxVel = 3.5;

const float turnFVoltage = 1.5;
const float moveFVoltage = 1;

float posTargetR, posTargetL;

// Past positions
float prev2PosRadR = 0;
float prev2PosRadL = 0;
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
void velDrive(int pwmPin, int dirPin, float velTarget, float maxV, float vel, float &velIntegral, float fVoltage) {
  float voltage;
  float velError = velTarget - vel;
  velIntegral += velError * Ts;


  float frictionVoltage = 0;
  if (velTarget > 0.01) {
    frictionVoltage = fVoltage;
  } else if (velTarget < -0.01) {
    frictionVoltage = -1 * fVoltage;
  }

  if (dirPin == RIGHT) {
    voltage = KpVELR * velError + KiVELR * velIntegral + frictionVoltage;
  }
  if (dirPin == LEFT) {
    voltage = KpVELL * velError + KiVELL * velIntegral + frictionVoltage;
  }

  // Make sure voltage is within range and drives motor
  voltage = constrain(voltage, -batteryV, batteryV);
  
  // Set direction if driving right
  if (voltage >= 0 && dirPin == RIGHT) {
    digitalWrite(RIGHT, LOW);
  } else if (dirPin == RIGHT) {
    digitalWrite(RIGHT, HIGH);
  }
  // Set direction if driving left opposite because the left
  // is mirrored
  if (voltage >= 0 && dirPin == LEFT) {
    digitalWrite(LEFT, HIGH);
  } else if (dirPin == LEFT) {
    digitalWrite(LEFT, LOW);
  }

  //int pwmLevel = (int)(255.0 * fabs(voltage) / batteryV);
  //analogWrite(pwmPin, pwmLevel);
  Timer1.pwm(pwmPin, (1023 * fabs(voltage) / batteryV));
}

// Function to drive and control the motor based on target position
void posDrive(int pwmPin, int dirPin, float posTarget, float maxV, float pos, float vel, float &posIntegral, float &velIntegral, float fVoltage) {
  float velTarget;
  float posError = posTarget - pos;
  posIntegral += posError * Ts;
  if (dirPin == RIGHT) {
    velTarget = KpPOSR * posError + KiPOSR * posIntegral;
  }
  if (dirPin == LEFT) {
    velTarget = KpPOSL * posError + KiPOSL * posIntegral;
  }
  velTarget = constrain(velTarget, -maxV, maxV);
  velDrive(pwmPin, dirPin, velTarget, maxV, vel, velIntegral, fVoltage);
}

void syncDrive(float posR, float posL, float targetR, float targetL, float &maxVelR, float &maxVelL, float baseMaxV) {
  float dirR, dirL;

  if (targetR >= 0) {
    dirR = 1.0;
  } else {
    dirR = -1.0;
  }

  if (targetL >= 0) {
    dirL = 1.0;
  } else {
    dirL = -1.0;
  }

  // Progress in the commanded direction
  float progressR = dirR * posR;
  float progressL = dirL * posL;

  float syncError = progressL - progressR;

  maxVelR = baseMaxV;
  maxVelL = baseMaxV;

  // Positive error means left wheel is ahead
  if (syncError > 0) {
    maxVelL -= KpSYNC * syncError;
  } else {
    maxVelR += KpSYNC * syncError;
  }

  maxVelR = constrain(maxVelR,  baseMaxV - 0.5, baseMaxV + 0.5);
  maxVelL = constrain(maxVelL,  baseMaxV - 0.5, baseMaxV + 0.5);
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

void turn(float turnDegrees, float baseMaxV) {
  // Zero the encoders so the drive starts from 0
  posR = 0;
  posL = 0;

  float prev2PosRadR, prevPosRadR, posRadR;
  float prev2PosRadL, prevPosRadL, posRadL;
  prev2PosRadR = prevPosRadR = posRadR = 0;
  prev2PosRadL = prevPosRadL = posRadL = 0;
  posIntegralR = posIntegralL = 0;
  velIntegralR = velIntegralL = 0;

  //set clockwise/counterclockwise
  posTargetL =  turnDegrees * radPerDeg;
  posTargetR = -turnDegrees * radPerDeg;

  // Restart the timer so the stop time counts from the start of the drive
  start_time_ms = last_time_ms = millis();
    
  float tolerance = 0.01;
  while (abs(posRadR - posTargetR) > tolerance ||
        abs(prevPosRadR - posTargetR) > tolerance ||
        abs(prev2PosRadR - posTargetR) > tolerance ||
        abs(posRadL - posTargetL) > tolerance ||
        abs(prevPosRadL - posTargetL) > tolerance ||
        abs(prev2PosRadL - posTargetL) > tolerance) {

    Serial.println("turning");

    // Find the time in seconds since last loop
    float t = (last_time_ms - start_time_ms) / 1000.0;
  
    // Turns counts into rad values and estimates velocity.
    posRadR = 2 * PI * posR / 3200.0;
    float velR = (posRadR - prevPosRadR) / (Ts);
    prev2PosRadR = prevPosRadR;
    prevPosRadR = posRadR;
    posRadL = -2 * PI * posL / 3200.0;
    float velL = (posRadL - prevPosRadL) / (Ts);
    prev2PosRadL = prevPosRadL;
    prevPosRadL = posRadL;

    float maxVelR, maxVelL;

    syncDrive(posRadR, posRadL, posTargetR, posTargetL,maxVelR, maxVelL, baseMaxV);

    // Call the position controllers for each wheel
    posDrive(PWM_RIGHT, RIGHT, posTargetR, maxVelR, posRadR,
      velR, posIntegralR, velIntegralR, turnFVoltage);
    posDrive(PWM_LEFT, LEFT, posTargetL, maxVelL, posRadL, 
      velL, posIntegralL, velIntegralL, turnFVoltage);
    
    Serial.print(t);
    Serial.print(", ");
    Serial.print(velL);
    Serial.print(", ");
    Serial.println(velR);


    while (millis() < last_time_ms + Ts_ms) {}
    last_time_ms = millis();
  }
  Timer1.pwm(PWM_RIGHT, (0));
  Timer1.pwm(PWM_LEFT, (0));
  return;
}

void move(float moveDistanceIn, float baseMaxV) {
  // Zero the encoders so the drive starts from 0
  posR = 0;
  posL = 0;

  float prev2PosRadR, prevPosRadR, posRadR;
  float prev2PosRadL, prevPosRadL, posRadL;
  prev2PosRadR = prevPosRadR = posRadR = 0;
  prev2PosRadL = prevPosRadL = posRadL = 0;
  posIntegralR = posIntegralL = 0;
  velIntegralR = velIntegralL = 0;

  // Set the drive targets
  posTargetR = moveDistanceIn * radPerIn;
  posTargetL = moveDistanceIn * radPerIn;

  // Restart the timer so the stop time counts from the start of the drive
  start_time_ms = last_time_ms = millis();
  float tolerance = 0.03;
  while (abs(posRadR - posTargetR) > tolerance ||
        abs(prevPosRadR - posTargetR) > tolerance ||
        abs(prev2PosRadR - posTargetR) > tolerance ||
        abs(posRadL - posTargetL) > tolerance ||
        abs(prevPosRadL - posTargetL) > tolerance ||
        abs(prev2PosRadL - posTargetL) > tolerance) {
          
    Serial.println("moving");
    // Find the time in seconds since last loop
    float t = (last_time_ms - start_time_ms) / 1000.0;
  
    // Turns counts into rad values and estimates velocity.
    posRadR = 2 * PI * posR / 3200.0;
    float velR = (posRadR - prevPosRadR) / (Ts);
    prev2PosRadR = prevPosRadR;
    prevPosRadR = posRadR;

    posRadL = -2 * PI * posL / 3200.0;
    float velL = (posRadL - prevPosRadL) / (Ts);
    prev2PosRadL = prevPosRadL;
    prevPosRadL = posRadL;

    float maxVelR, maxVelL;
    // Call the position controllers for each wheel
    syncDrive(posRadR, posRadL, posTargetR, posTargetL,maxVelR, maxVelL, baseMaxV);

    posDrive(PWM_RIGHT, RIGHT, posTargetR, maxVelR, posRadR, 
      velR, posIntegralR, velIntegralR, moveFVoltage);
    posDrive(PWM_LEFT, LEFT, posTargetL, maxVelL, posRadL,
      velL, posIntegralL, velIntegralL, moveFVoltage);

    while (millis() < last_time_ms + Ts_ms) {}
    last_time_ms = millis();
  }
  Timer1.pwm(PWM_RIGHT, (0));
  Timer1.pwm(PWM_LEFT, (0));
  return;
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
  
  Timer1.frequency(0.5);

  // Inialize start and last time to same value
  start_time_ms = last_time_ms = millis();
}

void loop() {
  // Sets the target rotations in radians based on the bits recieved
  delay(8000);
  turn(1080,2.5);
  move(180,6);




  /*
  Timer1.pwm(PWM_RIGHT, (1023 * 1.1 / batteryV));
  digitalWrite(LEFT, HIGH);
  Timer1.pwm(PWM_LEFT, (1023 * 1.1 / batteryV));
  delay(20000);
 */

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
  /*
  Serial.print(t);
  Serial.print(", ");
  Serial.print(velL);
  Serial.print(", ");
  Serial.println(velR);
 */
  Serial.println("adsfhiasdufh");
  // Waits set time to delay next loop cycle
  Timer1.pwm(PWM_RIGHT, (0));
  Timer1.pwm(PWM_LEFT, (0));
  while (true);
}
