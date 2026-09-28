//Assignment 2 question 2a motor control
//Set pin numbers
const int ENABLE = 4;
const int RIGHT = 8;
const int LEFT = 7;
const int PWM_RIGHT = 10;
const int APIN = 2;
const int BPIN = 3;

//set ISR variables
volatile long count = 0;
volatile int lastA;
volatile int lastB;

//Constants used for controls and calculations
const float pi = 3.14159265;
const float battery_V = 7.6;
const float desired_vel = 12.0;
const float Kp = 0.5;
const float Ki = 2;

//Timing variables
const unsigned long Ts_ms = 10;
const float Ts = Ts_ms / 1000.0;
const float run_time = 3.0;
unsigned long start_time_ms, last_time_ms;
float prev_pos_rad = 0;
float integral = 0;

//Encoder ISR that is used to detect and count direction of motor encoder when turned
void encoderISR() {
int thisA = digitalRead(APIN);
int thisB = digitalRead(BPIN);
if (lastA != thisA || lastB != thisB) {
if ((lastA == 0 && lastB == 0) && (thisA == 0 && thisB == 1)) ++count;
if ((lastA == 0 && lastB == 0) && (thisA == 1 && thisB == 0)) --count;
if ((lastA == 0 && lastB == 1) && (thisA == 0 && thisB == 0)) --count;
if ((lastA == 0 && lastB == 1) && (thisA == 1 && thisB == 1)) ++count;
if ((lastA == 1 && lastB == 0) && (thisA == 0 && thisB == 0)) ++count;
if ((lastA == 1 && lastB == 0) && (thisA == 1 && thisB == 1)) --count;
if ((lastA == 1 && lastB == 1) && (thisA == 0 && thisB == 1)) --count;
if ((lastA == 1 && lastB == 1) && (thisA == 1 && thisB == 0)) ++count;
}
lastA = thisA;
lastB = thisB;
}

//function to drive the motor that is called in loop after voltage is set.
//Handles -/+ voltages for forward and backwards turning.
void drive(float voltage) {
if (voltage >= 0) {
digitalWrite(RIGHT, HIGH);
digitalWrite(LEFT, LOW);
} else {
digitalWrite(RIGHT, LOW);
digitalWrite(LEFT, HIGH);
}
int pwm = (int)(255.0 * fabs(voltage) / battery_V);
analogWrite(PWM_RIGHT, pwm);
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
digitalWrite(ENABLE, HIGH);
pinMode(APIN, INPUT_PULLUP);
pinMode(BPIN, INPUT_PULLUP);
lastA = digitalRead(APIN);
lastB = digitalRead(BPIN);

//set interupts
attachInterrupt(digitalPinToInterrupt(APIN), encoderISR, CHANGE);
attachInterrupt(digitalPinToInterrupt(BPIN), encoderISR, CHANGE);
//inialize start and last time to same value
start_time_ms = last_time_ms = millis();
}

//Main loop
void loop() {

//find the time in seconds since last loop
float t = (last_time_ms - start_time_ms) / 1000.0;

//stops motor and program when run time is up
if (t > run_time) {
analogWrite(PWM_RIGHT, 0);
return;
}

//disables interupts so count can be read and not be changed by isr midread
noInterrupts();
long c = count;
interrupts();

//turns counts into rad values and estimates velocity.
float pos_rad = -2 * pi * c / 3200.0;
float vel = (pos_rad - prev_pos_rad) / Ts;
prev_pos_rad = pos_rad;

//Finds error from expected and finds next voltage
float error = desired_vel - vel;
integral += error * Ts;
float voltage = Kp * error + Ki * integral;

//make sure voltage is within range and drives motor
voltage = constrain(voltage, -battery_V, battery_V);
drive(voltage);

//prints data
Serial.print(t);
Serial.print(", ");
Serial.print(voltage);
Serial.print(", ");
Serial.println(vel);

//waits set time to delay next loop cycle
while (millis() < last_time_ms + Ts_ms) {}
last_time_ms = millis();
}