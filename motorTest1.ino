 #include "CytronMotorDriver.h"

// Configure the motor driver.
CytronMD motorL(PWM_DIR, 3, 2);  // PWM 1 = Pin 3, DIR 1 = Pin 2.
CytronMD motorR(PWM_DIR, 5, 4); // PWM 2 = Pin 5, DIR 2 = Pin 4.
CytronMD motorS(PWM_DIR, 6, 7); // PWM 3 = Pin 6, DIR 3 = Pin 7.

// Configure State Machine (For Relay)
bool remoteControl = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, LOW);
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 256; i++) {
    motorL.setSpeed(i);   // Motor 1 increment.
    motorR.setSpeed(i);   // Motor 1 increment.
    delay(20);
  } 
  delay(1000);
  for (int i = 255; i >= 0; i--) {
    motorL.setSpeed(i);   // Motor 1 decrement.
    motorR.setSpeed(i);   // Motor 1 decrement.
    delay(20);
  }
  delay(1000);
  for (int i = 0; i > -256; i--) {
    motorL.setSpeed(i);   // Motor 1 decrement.
    motorR.setSpeed(i);   // Motor 1 decrement.
    delay(20);
  }
  delay(1000);
  for (int i = -255; i <= 0; i++) {
    motorL.setSpeed(i);   // Motor 1 decrement.
    motorR.setSpeed(i);   // Motor 1 decrement.
    delay(20);
  }
  digitalWrite(13, HIGH);
  delay(10000);
}
