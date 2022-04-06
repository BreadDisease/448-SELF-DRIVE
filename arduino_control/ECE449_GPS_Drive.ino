/*
   Copyright (C) 2022 Matt Aaron, Taylor Owens, and Aidan Quimby.
*/
#include <AutoPID.h>
#include <CytronMotorDriver.h>
#include <TinyGPSPlus.h>
#include <Adafruit_HMC5883_U.h>

/* =======================
   Configuration
   ======================= */
// PID parameters
#define PID_CYCLE_TIME_MS 200
#define PID_DRIVE_KP 4
#define PID_DRIVE_KI 8
#define PID_DRIVE_KD 0

// Magnetometer calibration
#define MAG_X_MIN  -30.82
#define MAG_X_MAX   19.27
#define MAG_Y_MIN  -36.00
#define MAG_Y_MAX   15.36
#define MAG_DECLINATION -0.07214  // -5º 52' (Oxford, OH)

// Pin assignments
#define PIN_ENCODER_LEFT  19
#define PIN_ENCODER_RIGHT 18

#define TURNING_RADIUS 2.06  // meters
#define WHEEL_CIRCUMFERENCE 1.117
#define ONE_REV_TICKS 17

// END OF CONFIGURATION
// =======================

enum ControlMode { STOCK, AUTO };
enum State { RUN, STOP };
struct Waypoint {
  double lat;
  double lng;
};

int currWaypointIdx = 0;
Waypoint route[] = {
  { 39.5102767680703, -84.7326471498825 },
  { 39.51027632936558, -84.73254972346685 },
  { 39.51027578137975, -84.73247258665262 },
  { 39.51027528137978, -84.73240286272696 },
  { 39.510276180133495, -84.73233448964471 },
  { 39.51027546610668, -84.73226528850194 },
  { 39.510272602721564, -84.7321575134061 },
  { 39.5102705293654, -84.73206027008631 },
  { 39.51026922224198, -84.73192867148735 },
  { 39.51026919871786, -84.73180867131127 },
  { 39.510266453780645, -84.73170221686365 },
  { 39.51026753055359, -84.7316162288247 },
  { 39.5102622583931, -84.7315089026633 }
};

/* =======================
   State Variables
   ======================= */
State state = STOP;              // Current state
TinyGPSLocation startLocation;   // Location recorded when state changes

/* =======================
   Motor Drivers
   ======================= */
namespace Motor {
CytronMD L(PWM_DIR, 3, 2);
CytronMD R(PWM_DIR, 5, 4);
CytronMD S(PWM_DIR, 6, 7);
}

/* =======================
   GPS and Compass
   ======================= */
TinyGPSPlus gps;
namespace Compass {
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
double relHeading = 0;
}

/* =======================
   PID Variables
   ======================= */
namespace Encoder {
volatile double ticksL;
volatile double ticksR;

volatile double totalTicksL;
volatile double totalTicksR;
}

namespace PID {
unsigned long lastCycleTime = 0;
double setpointL = 2;
double setpointR = 2;
double speedL = 0;
double speedR = 0;
double speedS = 0;

// Steering
int maxPos = 0;
int minPos = 0;
int ctrPos = 0;
double actSteerPos = 0;
double relSteerPos = 0;
double setpointS = 0;
double targetHeading = 0;
const double setpointC = 0;  // This needs to be a variable for the library to work

// Left and right drive PIDs
AutoPID L(&Encoder::ticksL,     &setpointL, &speedL,   0, 255, PID_DRIVE_KP, PID_DRIVE_KI, PID_DRIVE_KD);
AutoPID R(&Encoder::ticksR,     &setpointR, &speedR,   0, 255, PID_DRIVE_KP, PID_DRIVE_KI, PID_DRIVE_KD);

// Steering PID
// Setpoint is set to zero because we work with relative heading
AutoPID S(&actSteerPos, &setpointS, &speedS, -255,  255, 5, 8, 0);

// Compass PID
AutoPID C(&Compass::relHeading, &setpointC, &relSteerPos, -110, 110, 1, 0, 0);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);

  // Initialize encoders
  pinMode(PIN_ENCODER_LEFT,  INPUT_PULLUP);
  pinMode(PIN_ENCODER_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT),  encoderLeftISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), encoderRightISR, RISING);

  // Configure control relay pins
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);

  // Set up PID cycle time to be controlled manually
  PID::L.setTimeStep(1);
  PID::R.setTimeStep(1);
  PID::S.setTimeStep(1);
  PID::C.setTimeStep(1);

  // Set control mode to AUTO
  setControlMode(AUTO);

  // Initialize magnetometer
  Compass::mag.begin();

  // Calibrate steering
  calibrateSteering();

  // Wait for GPS to acquire fix
  while (!gps.hdop.isValid() || gps.hdop.hdop() > 1.35 || !gps.location.isValid()) {
    getGPSData();
    Serial.println(gps.hdop.hdop());
  }

  // Wait for Pi to boot and login
  bool credPrompt = false;
  char serialBuff[7];
  while (!credPrompt) {
    while (Serial3.available()) {
      char in = (char) Serial3.read();
      Serial.print(in);
      serialBuff[0] = serialBuff[1];
      serialBuff[1] = serialBuff[2];
      serialBuff[2] = serialBuff[3];
      serialBuff[3] = serialBuff[4];
      serialBuff[4] = serialBuff[5];
      serialBuff[5] = serialBuff[6];
      serialBuff[6] = in;
    }

    if (serialBuff[0] == 'l' && serialBuff[5] == ':') {
      credPrompt = true;
      break;
    }
  }

  if (credPrompt) {
    Serial3.println("pi");
    delay(1000);
    Serial3.println("raspberry");
  }

  // Fix acquired, set state to RUN
  setState(RUN);
}

void loop() {
  getGPSData();
  getCompassData();
  runPID();

  while (Serial3.available()) {
    Serial.print((char) Serial3.read());
  }

  while (Serial.available()) {
    Serial3.print((char) Serial.read());
  }


  switch (state) {
    case RUN:
      PID::setpointL = 2;
      PID::setpointR = 2;

      // Differential drive
      // Adjust rear wheel differential to correct heading
      if (Compass::relHeading > 0) {
        // Turn left (right wheel is faster)
        // PID::setpointL -= 1;
        // PID::setpointR += 1;
      } else if (Compass::relHeading < 0) {
        // Turn right (left wheel is faster)
        // PID::setpointL += 1;
        // PID::setpointR -= 1;
      }

      PID::targetHeading     = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), route[currWaypointIdx].lat, route[currWaypointIdx].lng);
      double distToWaypoint  = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), route[currWaypointIdx].lat, route[currWaypointIdx].lng);
      if (distToWaypoint <= 1.75) {
        currWaypointIdx++;

        if (currWaypointIdx == 13) {
          state = STOP;
          PID::setpointL = 0;
          PID::setpointR = 0;
        }
      }
      break;

    case STOP:
      Motor::L.setSpeed(0);
      Motor::R.setSpeed(0);
      while (true) {}
      PID::setpointL = 0;
      PID::setpointR = 0;
      break;
  }
}

/**
   Runs the PID calculations at the appropriate time interval, resets the
   encoder ticks, and updates the motor speeds.
*/
void runPID() {
  if (millis() - PID::lastCycleTime > PID_CYCLE_TIME_MS) {
    PID::actSteerPos = analogRead(A12);
    PID::L.run();
    PID::R.run();
    PID::C.run();

    PID::setpointS = PID::relSteerPos + PID::ctrPos;
    PID::S.run();

    noInterrupts();
    Encoder::ticksL    = 0;
    Encoder::ticksR    = 0;
    PID::lastCycleTime = millis();
    interrupts();

    Motor::L.setSpeed(PID::speedL);
    Motor::R.setSpeed(PID::speedR);
    Motor::S.setSpeed(PID::speedS);
  }
}

/**
   Simple helper method to store the current location before changing states.

   \param[in] state The state to change to.
*/
void setState(State newState) {
  startLocation = gps.location;
  state         = newState;
}

/**
   Gets the current GPS data from Serial1 (pins 18 and 19).
*/
void getGPSData() {
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }
}

/**
   Gets the current compass data from the HMC5883L and stores it in
   Compass::heading to be used by the PID controller.

   See: https://reprage.com/post/measuring-your-direction-with-arduino
*/
void getCompassData() {
  sensors_event_t event;
  Compass::mag.getEvent(&event);

  float heading = atan2((event.magnetic.y - ((MAG_Y_MAX + MAG_Y_MIN) / 2.0)), (event.magnetic.x - ((MAG_X_MAX + MAG_X_MIN) / 2.0)));
  heading += MAG_DECLINATION;

  if (heading < 0) {
    // Correct for when signs are reversed.
    heading += 2 * PI;
  } else if (heading > 2 * PI) {
    // Check for wrap around due to addition of declination.
    heading -= 2 * PI;
  }

  float headingDegrees = heading * 180 / M_PI;  // Absolute heading in degrees, from 0 to 360.
  // Incorporate GPS heading
  if (gps.course.isValid() && gps.speed.isValid() && gps.speed.kmph() > 1) {  // Faster than 1 km/h
    float gpsHeadingDegrees = gps.course.deg();
    headingDegrees = (gpsHeadingDegrees * 0.20) + (headingDegrees * 0.80);
  }

  // For PID steering, we need to convert this absolute heading to a relative
  // heading in the range of -180 to 180, such that the setpoint can be zero.
  Compass::relHeading = headingDegrees - PID::targetHeading;
  if (Compass::relHeading > 180) {
    Compass::relHeading -= 360;
  } else if (Compass::relHeading < -180) {
    Compass::relHeading += 360;
  }
}

/**
   Helper method to set the control mode. Sets all of the control relay pins to
   either high or low, depending on the desired state.
*/
void setControlMode(ControlMode mode) {
  digitalWrite(A0, (mode == AUTO) ? LOW : HIGH);
  digitalWrite(A1, (mode == AUTO) ? LOW : HIGH);
  digitalWrite(A2, (mode == AUTO) ? LOW : HIGH);
  digitalWrite(A3, (mode == AUTO) ? LOW : HIGH);
  digitalWrite(A4, (mode == AUTO) ? LOW : HIGH);
  digitalWrite(A5, (mode == AUTO) ? LOW : HIGH);
}

/**
   Interrupt service routine for the left encoder.
*/
void encoderLeftISR() {
  Encoder::ticksL += 1;
  Encoder::totalTicksL += 1;
}

/**
   Interrupt service routine for the right encoder.
*/
void encoderRightISR() {
  Encoder::ticksR += 1;
  Encoder::totalTicksR += 1;
}

void calibrateSteering() {
  Motor::S.setSpeed(255);
  delay(1000);
  // read right
  PID::maxPos = analogRead(A12);
  Motor::S.setSpeed(0);
  delay(250);
  Motor::S.setSpeed(-255);
  delay(1000);
  // read left
  PID::minPos = analogRead(A12);
  Motor::S.setSpeed(0);
  delay(250);

  // Calculate center
  PID::ctrPos = (PID::maxPos + PID::minPos) / 2;
  Serial.println(PID::minPos);
  Serial.println(PID::ctrPos);
  Serial.println(PID::maxPos);
}
