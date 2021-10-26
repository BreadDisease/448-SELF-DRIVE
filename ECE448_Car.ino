/*
   Aidan Quimby
   Taylor Owens
   Matt Aaron
   ECE 448
   Self Driving Car Driving Program
   10/26/2021
*/

#include "CytronMotorDriver.h"
#include <ELECHOUSE_CC1101.h>

// Configure the motor driver.
CytronMD motorL(PWM_DIR, 3, 2);  // PWM 1 = Pin 3, DIR 1 = Pin 2.
CytronMD motorR(PWM_DIR, 5, 4); // PWM 2 = Pin 5, DIR 2 = Pin 4.
CytronMD motorS(PWM_DIR, 6, 7); // PWM 3 = Pin 6, DIR 3 = Pin 7.

// RF variables
byte RX_buffer[3] = {0};
byte flag;

// Joystick variables
int x, y, sw;

// Boolean to state switch between stock remote control and our remote control
bool state = true;

// Pin to switch relay state
int relayPin = 7;

void getData() {
  ELECHOUSE_cc1101.ReceiveData(RX_buffer);
  x = RX_buffer[0] * 2;
  y = RX_buffer[1] * 2;
  sw = RX_buffer[2];
  if (sw == 0) {
    state = !state;
  }
  if (state) {
    digitalWrite(relayPin, LOW);
    drive();
  } else {
    digitalWrite(relayPin, HIGH);
  }
  drive();
  Serial.println(x);
  Serial.println(y);
  Serial.println(sw);
  ELECHOUSE_cc1101.SetReceive();
}

void drive() {
  Serial.println("Driving...");
  motorL.setSpeed(0);   // Drive left motor at x value
  motorR.setSpeed(0);   // Drive right motor at x value
  if (x <= 280) {
    motorL.setSpeed(x - 256);   // Drive left motor at x value
    motorR.setSpeed(x - 256);   // Drive right motor at x value
  } else if (x >= 240) {
    motorL.setSpeed(x - 256);   // Drive left motor at x value
    motorR.setSpeed(x - 256);   // Drive right motor at x value
  }
  if (y >= 280) {
    // motorL.setSpeed(x);   // Drive left motor at x value
    motorR.setSpeed(y - 256);   // Drive right motor at x value
  } else if (y <= 240) {
    motorL.setSpeed(y + 256);   // Drive left motor at x value
    // motorR.setSpeed(x);   // Drive right motor at x value
  }
}

void setup() {
  pinMode(relayPin, OUTPUT);
  Serial.begin(9600);
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.SetReceive();
}

void loop() {
  if (ELECHOUSE_cc1101.CheckReceiveFlag()) {
    getData();
  }
  //delay(250);
}
