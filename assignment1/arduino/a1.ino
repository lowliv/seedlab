// Name: Cameron MacMillan
// Class: EENG350 (Group 12)
//
// Code for encoder and LED cycle with gas and break button.
// To run install EnableInterrupt library to allow more than
// 2 interrupt pins and connect the pins according to the comments.
// Comment out either the interrupt encoder or the non-interrupt encoder
// so they don't interfere.
//
// also the programs assumes leds are setup in an active LOW configuration

#include <EnableInterrupt.h>
volatile int pos = 0;
volatile unsigned long prevEncTime = 0;
volatile unsigned long prevGasTime = 0;
volatile unsigned long prevBreakTime = 0;

int thisA;
int thisB;
int lastA;
int lastB;
volatile unsigned long waitTime = 1500;
int stepArray[8] = {5,6,7,8,5,6,7,8};
int step = 0;

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP); //clk
  pinMode(3, INPUT_PULLUP); //dt
  pinMode(4, INPUT_PULLUP); //sw
  pinMode(5, OUTPUT); // green LED 1
  pinMode(6, OUTPUT); // red LED 2
  pinMode(7, OUTPUT); // green LED 3
  pinMode(8, OUTPUT); // red LED 4
  pinMode(9, INPUT_PULLUP); // button
  pinMode(10, INPUT_PULLUP); // button
  enableInterrupt(2, encoderIsr, CHANGE); // CW because clk first
  enableInterrupt(9, gasButton, FALLING);
  enableInterrupt(10, breakButton, FALLING);

  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  
}

void loop() {
// Code for LEDs
  delay(waitTime);
  if (step == 8) {
    step=0;
  }
  if (step < 4) {
    digitalWrite(stepArray[step], LOW);
  }
  if (step >= 4) {
    digitalWrite(stepArray[step], HIGH);
  }
  step++;
// End Code for LEDs

// Code for non inturrupt encoder
/*
  thisA = digitalRead(2);
  thisB = digitalRead(3);
  if (lastA != thisA) {
    if (thisA == thisB) {
      pos++;
      Serial.println(pos);
    }
    if (thisA != thisB) {
      pos--;
      Serial.println(pos);
    }
  }
  lastA = thisA;
  lastB = thisB;
*/
// End non inturrupt encoder
}
// LED Button Inturrupts
void gasButton() {
  if (millis() - prevGasTime < 100 || waitTime == 20) { // waitTime is for not overflow
    return;
  }
  Serial.println(waitTime);
  waitTime = waitTime - 20;
  prevGasTime = millis();
}

void breakButton() {
  if (millis() - prevBreakTime < 100 || waitTime == 10000) { // waitTime is for notoverflow
    return;
  }
  Serial.println(waitTime);
  waitTime = waitTime + 20;
  prevBreakTime = millis();
}
// END LED Button ISR

// Encoder Interrupt code
void encoderIsr() {
  if (millis() - prevEncTime < 50) {
    return;
  }

  int clkPin = digitalRead(2);
  int dtPin = digitalRead(3);
  if (clkPin == dtPin) {
    pos++;
  } 
  else {
    pos--;
  }
  prevEncTime = millis();
  Serial.println(pos);
}