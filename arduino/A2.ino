// Name: Cameron MacMillan
// Class: EENG350 (Group 12)
//
// Code to get the position of the motors from the encoders. To run install EnableInterrupt
// library to allow for additional interrupt pins and connect the pins to match the comments.
// Uncomment the motor code for powered movement by default this program is setup for manually
// turning the motors
//
#include <EnableInterrupt.h>

const int ENABLE = 4;
const int LEFT = 7;
const int RIGHT = 8;
const int PWM_LEFT = 9;
const int PWM_RIGHT = 10;

const int clkPinR = 2; // Right motor encoder clk
const int dtPinR = 3; // Right motor encoder dt

const int clkPinL = 5; // Left motor encoder clk
const int dtPinL = 6; // Left motor encoder dt

volatile int posR = 0; // Right raw encoder counts
volatile int posL = 0; // Left raw encoder counts

volatile int lastAR, lastBR, lastAL, lastBL; // Previous encoder variables for Isr

// Variables for timing outputs
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;
unsigned long desired_Ts_ms = 10;

float pi = 3.14159;

// Encoder interrupt code right
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

// Encoder interrupt code left
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

void setup() {
  Serial.begin(115200); // Set the baud rate fast so that we can display the results

  // Setup motor pins
  pinMode(ENABLE, OUTPUT);
  pinMode(LEFT, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  digitalWrite(ENABLE, HIGH);
  
  // Setup encoder pins
  pinMode(clkPinR, INPUT_PULLUP);
  pinMode(dtPinR, INPUT_PULLUP); 
  pinMode(clkPinL, INPUT_PULLUP); 
  pinMode(dtPinL, INPUT_PULLUP); 
  // Set encoder starting states
  lastAL = digitalRead(clkPinR);
  lastBL = digitalRead(dtPinR);
  lastAL = digitalRead(clkPinL);
  lastBL = digitalRead(dtPinL);
  //Start interrupts
  enableInterrupt(2, encoderIsrR, CHANGE);
  enableInterrupt(3, encoderIsrR, CHANGE);
  enableInterrupt(5, encoderIsrL, CHANGE);
  enableInterrupt(6, encoderIsrL, CHANGE);

  last_time_ms = millis();
  start_time_ms = last_time_ms;
}

void loop() {

  //digitalWrite(LEFT, LOW);
  //analogWrite(PWM_LEFT, 128);

  //digitalWrite(RIGHT, HIGH);
  //analogWrite(PWM_RIGHT, 128);
  //delay(2000);

  // Variables for calculating position
  long posR_counts;
  long posL_counts;
  float posR_rad;
  float posL_rad;


  posR_counts = posR; // your encoder function that returns the current
  posR_rad= 2*pi*(float)posR_counts/3200; // position for motor right

  posL_counts = posL; // your encoder function that returns the current
  posL_rad= 2*pi*(float)posL_counts/3200; // position for motor left

  current_time = (float)(last_time_ms - start_time_ms)/1000;
  Serial.print(current_time);
  Serial.print(", ");
  Serial.print(posR_rad);
  Serial.print(", ");
  Serial.print(posL_rad);
  Serial.println("");
  while (millis() < last_time_ms + desired_Ts_ms) {
  //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();
}
