#include <Wire.h>

#define MY_ADDR 8

uint8_t reply = 0;

void setup() {
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);
  Wire.onRequest(request);
}

void loop() {
}

void receive() {
  Wire.read();

  if (Wire.available())
    reply = Wire.read() + 100;
}

void request() {
  Wire.write(reply);
}
