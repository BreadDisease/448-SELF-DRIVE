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
#define MAG_X_MIN  -29.00
#define MAG_X_MAX   14.82
#define MAG_Y_MIN  -28.18
#define MAG_Y_MAX   16.82
#define MAG_DECLINATION -0.07214  // -5º 52' (Oxford, OH)

// Pin assignments
#define PIN_ENCODER_LEFT  19
#define PIN_ENCODER_RIGHT 18

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
  { 39.50875325337, -84.73282501449 },
  { 39.50890219116, -84.73278601591 },
  { 39.50910077438, -84.73264562103 },
  { 39.50920555734, -84.73269097767 },
  { 39.50920864372, -84.73306127219 },
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
AutoPID C(&Compass::relHeading, &setpointC, &relSteerPos, -110, 110, 5, 0, 0);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial2.begin(115200);

  // Initialize encoders
  pinMode(PIN_ENCODER_LEFT,  INPUT);
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
  while (!gps.hdop.isValid() || gps.hdop.hdop() > 1.5 || !gps.location.isValid()) {
    getGPSData();
  }

  // Fix acquired, set state to RUN
  setState(RUN);
}

void loop() {
  getGPSData();
  getCompassData();
  runPID();


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
      if (distToWaypoint <= 3) {
        currWaypointIdx++;

        if (currWaypointIdx == 5) {
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

    Motor::L.setSpeed(PID::speedR);
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
    headingDegrees = (gpsHeadingDegrees * 0.85) + (headingDegrees * 0.15);
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
}

/**
   Interrupt service routine for the right encoder.
*/
void encoderRightISR() {
  Encoder::ticksR += 1;
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
