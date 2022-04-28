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

unsigned long prevGPSDataTime = 0;

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
  { 39.5102794, -84.7324875 },
  { 39.5102797, -84.7324403 },
  { 39.5102801, -84.7323864 },
  { 39.5102799, -84.7323426 },
  { 39.5102776, -84.7322907 },
  { 39.5102764, -84.7322428 },
  { 39.5102752, -84.7321896 },
  { 39.510275,  -84.7321418 },
  { 39.5102727, -84.7320832 },
  { 39.5102731, -84.732038 },
  { 39.5102736, -84.7319901 },
  { 39.5102741, -84.7319449 },
  { 39.5102724, -84.731889 },
  { 39.5102729, -84.7318445 },
  { 39.5102733, -84.7317947 },
  { 39.5102707, -84.7317475 },
  { 39.5102701, -84.7316949 },
  { 39.5102691, -84.7316501 },
  { 39.5102682, -84.7316025 },
  { 39.5102651, -84.7315523 },
  { 39.5102642, -84.7314994 },
  { 39.5102642, -84.7314499 },
  { 39.5102643, -84.7314004 },
  { 39.5102654, -84.7313522 },
  { 39.5102644, -84.7313013 },
  { 39.5102642, -84.7312575 },
  { 39.510264,  -84.7312109 },
  { 39.5102618, -84.7311631 },
  { 39.5102616, -84.7311099 },
  { 39.5102598, -84.7310586 },
  { 39.5102591, -84.7310101 },
  { 39.5102584, -84.7309602 },
  { 39.5102587, -84.7309077 }
};

/* =======================
   State Variables
   ======================= */
State state = STOP;  // Current state

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
int queueCnt = 0;
TinyGPSLocation queue[5];
Waypoint currLoc;
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

namespace Vision {
char buff[128];
int buffIdx;
bool isConfident = false;
double steeringCorrection = 0.0;
double steerAdjustOut = 0;
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
const double setpointV = 0;

// Left and right drive PIDs
AutoPID L(&Encoder::ticksL,     &setpointL, &speedL,   0, 255, PID_DRIVE_KP, PID_DRIVE_KI, PID_DRIVE_KD);
AutoPID R(&Encoder::ticksR,     &setpointR, &speedR,   0, 255, PID_DRIVE_KP, PID_DRIVE_KI, PID_DRIVE_KD);

// Steering PID
// Setpoint is set to zero because we work with relative heading
AutoPID S(&actSteerPos, &setpointS, &speedS, -255,  255, 5, 8, 0);

// Compass PID
AutoPID C(&Compass::relHeading, &setpointC, &relSteerPos, -110, 110, 1, 0, 0);

// Vision PID
AutoPID V(&Vision::steeringCorrection, &setpointV, &Vision::steerAdjustOut, -110, 110, 10, 1, 1);
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
  PID::V.setTimeStep(1);

  // Set control mode to AUTO
  setControlMode(AUTO);

  // Initialize magnetometer
  Compass::mag.begin();

  // Calibrate steering
  calibrateSteering();

  // Wait for GPS to acquire fix
  while (!gps.hdop.isValid() || gps.hdop.hdop() > 1 || !gps.location.isValid()) {
    getGPSData();
  }

  // Fix acquired, set state to RUN
  setState(RUN);
}

void loop() {
  // getGPSData();
  getCompassData();
  readVisionData();
  runPID();

  if (millis() - prevGPSDataTime > 1000) {
    Serial.print("GPS: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.println(gps.location.lng(), 6);
    prevGPSDataTime = millis();
    Serial.print("WAY: ");
    Serial.println(currWaypointIdx);
  }

  switch (state) {
    case RUN:
      PID::setpointL = 1.5;
      PID::setpointR = 1.5;

      PID::targetHeading     = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), route[currWaypointIdx].lat, route[currWaypointIdx].lng);
      double distToWaypoint  = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), route[currWaypointIdx].lat, route[currWaypointIdx].lng);
      if (distToWaypoint <= 1.75) {  // within 1.75 m of waypoint
        currWaypointIdx++;

        if (currWaypointIdx == 33) {  // reached end
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
    PID::V.run();

    Serial.print("PID: ");
    Serial.println(Vision::steerAdjustOut);

    // Check with Vision
    if (Vision::isConfident) {
      PID::setpointS = Vision::steerAdjustOut + PID::relSteerPos + PID::ctrPos;
    } else {
      PID::setpointS = PID::relSteerPos + PID::ctrPos;
    }

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
   Simple helper method to change states.

   \param[in] state The state to change to.
*/
void setState(State newState) {
  state = newState;
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
}

/**
   Interrupt service routine for the right encoder.
*/
void encoderRightISR() {
  Encoder::ticksR += 1;
}

void calibrateSteering() {
  Motor::S.setSpeed(255);
  delay(500);
  // read right
  PID::maxPos = analogRead(A12);
  Motor::S.setSpeed(0);
  delay(250);
  Motor::S.setSpeed(-255);
  delay(500);
  // read left
  PID::minPos = analogRead(A12);
  Motor::S.setSpeed(0);
  delay(250);
  Motor::S.setSpeed(255);
  delay(250);
  Motor::S.setSpeed(0);

  // Calculate center
  PID::ctrPos = (PID::maxPos + PID::minPos) / 2;
  Serial.println(PID::minPos);
  Serial.println(PID::ctrPos);
  Serial.println(PID::maxPos);
}

void readVisionData() {
  while (Serial3.available()) {
    char in = Serial3.read();
    if (in != '\n') {
      Vision::buff[Vision::buffIdx] = in;
      Vision::buffIdx++;
    } else {
      // Full message received
      Vision::buff[Vision::buffIdx] = '\0';

      // Parse message
      if (Vision::buff[0] == '1') {  // Trust the angle
        Vision::isConfident = true;
        char *angleBuff;
        angleBuff = strtok(Vision::buff, ",");
        angleBuff = strtok(NULL, ",");
        Vision::steeringCorrection = -1.0 * atof(angleBuff);
      } else {
        Vision::isConfident = false;
        PID::V.reset();
      }

      if (Vision::isConfident) {
        Serial.print("VIS: Confident! Correction: ");
        Serial.println(Vision::steeringCorrection);
      } else {
        Serial.println("VIS: Not confident.");
      }

      Vision::buffIdx = 0;
    }
  }
}
