/*
   Aidan Quimby
   Taylor Owens
   Matt Aaron
   ECE 448
   Self Driving Car Driving Program with Raspberry Pi Control
   11/05/2021
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SPI.h>
#include "RF24.h"
#include "CytronMotorDriver.h"

// Assign a unique ID to the compass
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Compass tolerance in which it doesn't turn
// if within this amount relative to desired heading
uint8_t tolerance = 5;

// Configure the motor driver.
CytronMD motorL(PWM_DIR, 3, 2);  // PWM 1 = Pin 3, DIR 1 = Pin 2.
CytronMD motorR(PWM_DIR, 5, 4); // PWM 2 = Pin 5, DIR 2 = Pin 4.
CytronMD motorS(PWM_DIR, 6, 7); // PWM 3 = Pin 6, DIR 3 = Pin 7.

// Pin to switch relay state
int relayPin = A0;

// Motor speed variables
int xSpeed, ySpeed;
int change = 1;
int turnSpeed = 100;

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
  uint8_t sw;
  uint8_t heading;
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

  /* Initialise the sensor */
  if (!mag.begin()) {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
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
    Serial.println("Heading = ");
    Serial.println(payload.heading);
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
    ySpeed = -turnSpeed;
    motorL.setSpeed(xSpeed);   // Drive left motor at x value
    motorR.setSpeed(xSpeed);   // Drive right motor at x value
    motorS.setSpeed(ySpeed);   // Drive steering motor at y value
  } else if (payload.steer == 4) {
    ySpeed = turnSpeed;
    motorL.setSpeed(xSpeed);   // Drive left motor at x value
    motorR.setSpeed(xSpeed);   // Drive right motor at x value
    motorS.setSpeed(ySpeed);   // Drive steering motor at y value
  } else if (payload.steer == 0) {
    ySpeed = 0;
    motorL.setSpeed(xSpeed);   // Drive left motor at x value
    motorR.setSpeed(xSpeed);   // Drive right motor at x value
    motorS.setSpeed(ySpeed);   // Drive steering motor at y value
  }
}

float getHeading() {
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = -0.07185;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180 / M_PI;

  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);

  delay(200);
}

void moveForward() {
  if (xSpeed <= 255 - change)) {
    xSpeed += change;
  }
  motorL.setSpeed(xSpeed);   // Drive left motor at x value
  motorR.setSpeed(xSpeed);   // Drive right motor at x value
}

void gpsDrive() {
  uint8_t heading = (uint8_t) getHeading();
  if (payload.heading > heading - tolerance && payload.heading < heading + tolerance) {
    moveForward();
  } else if (payload.heading < heading) {
    slowDownX();
    tankTurn(2);
  } else if (payload.heading > heading) {
    slowDownX();
    tankTurn(1);
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

void tankTurn(int turnDir) {
  if (turnDir == 1) {
    motorL.setSpeed(-turnSpeed);
    motorR.setSpeed(turnSpeed);
  } else if (turnDir == 2) {
    motorL.setSpeed(turnSpeed);
    motorR.setSpeed(-turnSpeed);
  } else if (turnDir == 0) {
    motorL.setSpeed(0);
    motorR.setSpeed(0);
  }
}

void drive() {
  if (payload.sw == 0) {
    xSpeed = 0;
    ySpeed = 0;
    digitalWrite(relayPin, HIGH);
  } else if (payload.sw == 1) {
    digitalWrite(relayPin, LOW);
    // Serial.println("Driving...");
    changeXSpeed();
    changeYSpeed();
  } else if (payload.sw == 2) {
    digitalWrite(relayPin, LOW);
    gpsDrive();
  }
}
