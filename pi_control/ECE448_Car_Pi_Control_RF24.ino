/*
   Aidan Quimby
   Taylor Owens
   Matt Aaron
   ECE 448
   Self Driving Car Driving Program with Raspberry Pi Control
   11/05/2021
*/

#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "CytronMotorDriver.h"

// Configure the motor driver.
CytronMD motorL(PWM_DIR, 3, 2);  // PWM 1 = Pin 3, DIR 1 = Pin 2.
CytronMD motorR(PWM_DIR, 5, 4); // PWM 2 = Pin 5, DIR 2 = Pin 4.
CytronMD motorS(PWM_DIR, 6, 7); // PWM 3 = Pin 6, DIR 3 = Pin 7.

// Pin to switch relay state
int relayPin = A0;

// Motor speed variables
int xSpeed, ySpeed;
int change = 1;
int tankSpeed = 60;

// Boolean to state switch between stock remote control and our remote control
bool state = true;

/*
   Pi Variables
   dir is 0 for idle, 1 for forward, 2 for backwards
   steer is 0 for straight, 1 for hard left, 2 for hard right, 3 for soft left, 4 for soft right
*/
struct Payload {
  uint8_t dir;
  uint8_t steer;
  uint8_t speedVal;
  bool    sw;
};

// Initialize payload
Payload payload;

// Instantiate an object for the nRF24L01 transceiver
RF24 radio(9, 10); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Address to identify the transceivers
uint64_t address = 0x7878787878LL;

void setup() {
  pinMode(relayPin, OUTPUT);

  // init serial Port for debugging
  Serial.begin(115200); Serial.println();
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} // hold in infinite loop
  }

  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // Save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(4); // 4 bytes

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address); // using pipe 1
  radio.startListening(); // put radio in RX mode
  radio.setPayloadSize(sizeof(payload));

  Serial.println("Finished Setup.");
}

void loop() {
  uint8_t pipe;
  if (radio.available(&pipe)) {             // is there a payload? get the pipe number that recieved it
    radio.read(&payload, sizeof(payload));
    Serial.print("Direction = ");
    Serial.println(payload.dir);
    Serial.print("Steering = ");
    Serial.println(payload.steer);
    Serial.print("Speed Value = ");
    Serial.println(payload.speedVal);
    Serial.print("Switch = ");
    Serial.println(payload.sw);
    Serial.println();
  }
  drive();
}

void changeXSpeed() {
  if (payload.dir == 1 && xSpeed <= (payload.speedVal - change)) {
    xSpeed += change;
  } else if (payload.dir == 2 && xSpeed >= (-payload.speedVal + change)) {
    xSpeed -= change;
  } else if (payload.dir == 0) {
    slowDownX();
  }
}

void changeYSpeed() {
  if (payload.steer == 1 || payload.steer == 2) {
    tankTurn(payload.steer);
  } else if (payload.steer == 3) {
    ySpeed = -payload.speedVal / 2;
    motorL.setSpeed(xSpeed);   // Drive left motor at x value
    motorR.setSpeed(xSpeed);   // Drive right motor at x value
    motorS.setSpeed(ySpeed);   // Drive steering motor at y value
  } else if (payload.steer == 4) {
    ySpeed = payload.speedVal / 2;
    motorL.setSpeed(xSpeed);   // Drive left motor at x value
    motorR.setSpeed(xSpeed);   // Drive right motor at x value
    motorS.setSpeed(ySpeed);   // Drive steering motor at y value
  } else if (payload.steer == 0) {
    slowDownY();
    motorL.setSpeed(xSpeed);   // Drive left motor at x value
    motorR.setSpeed(xSpeed);   // Drive right motor at x value
    motorS.setSpeed(ySpeed);   // Drive steering motor at y value
  }
}

void slowDownX() {
  if (xSpeed > 30) {
    xSpeed -= change;
  } else if (xSpeed < -30) {
    xSpeed += change;
  } else {
    xSpeed = 0;
  }
}

void slowDownY() {
  //  if (ySpeed > 30) {
  //    ySpeed -= change;
  //  } else if (ySpeed < -30) {
  //    ySpeed += change;
  //  } else {
  //    ySpeed = 0;
  //  }
  ySpeed = 0;
}

void tankTurn(int turnDir) {
  if (turnDir == 1) {
    motorL.setSpeed(-tankSpeed);
    motorR.setSpeed(tankSpeed);
  } else if (turnDir == 2) {
    motorL.setSpeed(tankSpeed);
    motorR.setSpeed(-tankSpeed);
  } else if (turnDir == 0) {
    motorL.setSpeed(0);
    motorR.setSpeed(0);
  }
}

void drive() {
  if (payload.sw == 1) {
    state = !state;
    payload.sw = 0;
  }
  if (state) {
    digitalWrite(relayPin, LOW);
    // Serial.println("Driving...");
    changeXSpeed();
    changeYSpeed();
  } else {
    xSpeed = 0;
    ySpeed = 0;
    digitalWrite(relayPin, HIGH);
  }
}
