/*
   Aidan Quimby
   Taylor Owens
   Matt Aaron
   ECE 448
   Self Driving Car Driving Program
   11/05/2021
*/

#include "CytronMotorDriver.h"
#include <ELECHOUSE_CC1101.h>

// Configure the motor driver.
CytronMD motorL(PWM_DIR, 3, 2);  // PWM 1 = Pin 3, DIR 1 = Pin 2.
CytronMD motorR(PWM_DIR, 5, 4); // PWM 2 = Pin 5, DIR 2 = Pin 4.
CytronMD motorS(PWM_DIR, 6, 7); // PWM 3 = Pin 6, DIR 3 = Pin 7.

// Motor speed variables
int xSpeed, ySpeed;
int change = 30;
int deadzone = 16;

// RF variables
byte RX_buffer[3] = {0};
byte flag;

// Joystick variables
int x, y, sw;

// Boolean to state switch between stock remote control and our remote control
bool state = true;

// Pin to switch relay state
int relayPin = A0;

void getData() {
  ELECHOUSE_cc1101.ReceiveData(RX_buffer);
  // We double x and y here because the motor library operates between -256 and 256 (512 range)
  // While our joystick gives us values between 0 and 255 (256 range)
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
    xSpeed = 0;
    ySpeed = 0;
    digitalWrite(relayPin, HIGH);
  }
  Serial.println(x);
  Serial.println(y);
  Serial.println(sw);
  ELECHOUSE_cc1101.SetReceive();
}

void changeXSpeed() {
  // 262 is double the resting x position of the joystick
  // Subtracting it lets us zero the variable
  int absX = x - 262;
  if (absX > deadzone && xSpeed <= 256 - change) {
    xSpeed += change;
  } else if (absX < -deadzone && xSpeed >= -(256 - change)) {
    xSpeed -= change;
  } else if (absX <= deadzone || absX >= -deadzone) {
    slowDown();
  }
}

void changeYSpeed() {
  // 254 is double the resting y position of the joystick
  // Subtracting it lets us zero the variable
  int absY = y - 254;
  if (absY > deadzone) {
    ySpeed = 175;
  } else if (absY < -deadzone) {
    ySpeed = -175;
  } else {
    ySpeed = 0;
  }
}

void slowDown() {
  if (xSpeed > 30) {
    xSpeed -= change;
  } else if (xSpeed < -30) {
    xSpeed += change;
  } else {
    xSpeed = 0;
  }
}

void drive() {
  Serial.println("Driving...");
  changeXSpeed();
  changeYSpeed();
  motorL.setSpeed(xSpeed);   // Drive left motor at x value
  motorR.setSpeed(xSpeed);   // Drive right motor at x value
  motorS.setSpeed(ySpeed);   // Drive steering motor at y value
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
}
