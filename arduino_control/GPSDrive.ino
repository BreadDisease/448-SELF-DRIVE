// Copyright 2022 Matt Aaron

#include <TinyGPSPlus.h>
#include <CytronMotorDriver.h>

// State values
#define STATE_ESTOP          0
#define STATE_ACQUIRE_LOCK   1
#define STATE_COURSE_WARMUP  2
#define STATE_TRAVEL_ROUTE   3
#define STATE_CHG_COURSE      4
#define STATE_ROUTE_COMPLETE 5

// Motor speed values
#define SPEED_NORMAL 130
#define SPEED_LOW    80
#define SPEED_HIGH   255
#define SPEED_DELTA  5
#define SPEED_BOOST  200
#define SPEED_STOP   0

// Motor speed conditions used for correcting course
#define SPEED_EQUAL       0
#define SPEED_LEFT_BOOST  1
#define SPEED_RIGHT_BOOST 2

// Steering adjustment strength
#define STEERING_KICK_STRENGTH 75

// Struct to store waypoint data
struct Waypoint {
  double lat;
  double lng;
};

// Initialize motor driver library
CytronMD motorL(PWM_DIR, 3, 2);
CytronMD motorR(PWM_DIR, 5, 4);
CytronMD motorS(PWM_DIR, 6, 7);

// Initialize GPS library
TinyGPSPlus gps;

// State variables
byte state = STATE_ACQUIRE_LOCK;
unsigned long stateMillis;  // Millis the current state took effect

// Motor speed variables
byte speedState  = SPEED_EQUAL;
byte targetSpeed = SPEED_NORMAL;
// These are internal values
byte speedL = 0;
byte speedR = 0;

// Navigation variables
int currWaypointIdx = 0;
float course;
Waypoint route[] = {
  { 39.510275712407, -84.732652906215 },
  { 39.510269504350, -84.731724861899 }
};

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(A0, OUTPUT);
  digitalWrite(A0, LOW);

  motorL.setSpeed(0);
  motorR.setSpeed(0);
  motorS.setSpeed(0);
}

void loop() {
  readGPSData();
  course = gps.course.deg();
  double courseToWaypoint = TinyGPSPlus::courseTo(
                              gps.location.lat(),
                              gps.location.lng(),
                              route[currWaypointIdx].lat,
                              route[currWaypointIdx].lng
                            );
  double distToWaypoint = TinyGPSPlus::distanceBetween(
                            gps.location.lat(),
                            gps.location.lng(),
                            route[currWaypointIdx].lat,
                            route[currWaypointIdx].lng
                          );

  switch (state) {
    case STATE_ACQUIRE_LOCK:
      if (gps.hdop.isValid() && gps.hdop.hdop() <= 1.5 && gps.location.isValid()) {
        state = STATE_COURSE_WARMUP;
        stateMillis = millis();
      }
      break;

    case STATE_COURSE_WARMUP:
      targetSpeed = SPEED_LOW;
      // Drive straight for 5 seconds to get accurate course
      if (gps.course.isValid() && (millis() - stateMillis) > 5000) {
        state = STATE_TRAVEL_ROUTE;
      }
      drive();
      break;

    case STATE_TRAVEL_ROUTE:
      // Make sure HDOP is acceptable and that the current location is valid
      if (gps.hdop.hdop() >= 2.2 || !gps.location.isValid()) {
        state = STATE_ESTOP;
        halt();
        break;
      }

      // Lower speed nearing course change
      if (distToWaypoint < 5) {  // Slow down within 5 meters
        targetSpeed = SPEED_LOW;
      } else {
        targetSpeed = SPEED_NORMAL;
      }

      if (distToWaypoint < 1.5) {  // Assume we've made it (within 1.5 meters)
        currWaypointIdx++;
        if (currWaypointIdx < (sizeof(route) / sizeof(route[0]))) {  // Check if there's another waypoint
          currWaypointIdx++;
          // Calculate course to next waypoint
          state = STATE_CHG_COURSE;
          break;
        } else {
          state = STATE_ROUTE_COMPLETE;  // Yay!
          break;
        }
      } else {
        // Adjust course
        if (courseToWaypoint - course > 2) {  // Nudge right
          speedState = SPEED_LEFT_BOOST;
          kickSteering(STEERING_KICK_STRENGTH);  // Kick steering right
        } else if (courseToWaypoint - course < 2) {  // Nudge left
          speedState = SPEED_RIGHT_BOOST;
          kickSteering(-STEERING_KICK_STRENGTH);  // Kick steering left
        } else {
          speedState = SPEED_NORMAL;  // Go straight
        }
      }
      break;

    case STATE_CHG_COURSE:
      if (courseToWaypoint - course > 0) {  // Turn right
        turnSteering(100);
      } else if (courseToWaypoint - course < 0) {  // Turn left
        turnSteering(-100);
      }
      state = STATE_TRAVEL_ROUTE;
      break;

    case STATE_ROUTE_COMPLETE:
      // All done
      targetSpeed = SPEED_STOP;
      speedState  = SPEED_EQUAL;
      drive();
      break;

    case STATE_ESTOP:
      halt();
      break;
  }
}

void readGPSData() {
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }
}

void drive() {
  switch (speedState) {
    case SPEED_EQUAL:
      if (speedL < targetSpeed) {
        speedL += SPEED_DELTA;
      } else if (speedL > targetSpeed) {
        speedL -= SPEED_DELTA;
      }

      if (speedR < targetSpeed) {
        speedR += SPEED_DELTA;
      } else if (speedR > targetSpeed) {
        speedR -= SPEED_DELTA;
      }
      break;

    case SPEED_LEFT_BOOST:
      if (speedL < SPEED_BOOST) {
        speedL += SPEED_DELTA;
      } else if (speedL > SPEED_BOOST) {
        speedL -= SPEED_DELTA;
      }

      speedR = targetSpeed;
      break;

    case SPEED_RIGHT_BOOST:
      if (speedR < SPEED_BOOST) {
        speedR += SPEED_DELTA;
      } else if (speedR > SPEED_BOOST) {
        speedR -= SPEED_DELTA;
      }

      speedL = targetSpeed;
      break;
  }

  motorL.setSpeed(speedL);
  motorR.setSpeed(speedR);
}

void kickSteering(int val) {
  motorS.setSpeed(val);
  delay(100);
  motorS.setSpeed(0);
}

void turnSteering(int val) {
  motorS.setSpeed(val);
  delay(1500);  // just some value to for turn duration
  motorS.setSpeed(0);
}

void halt() {
  speedL = 0;
  speedR = 0;
  motorL.setSpeed(0);
  motorR.setSpeed(0);
  motorS.setSpeed(0);
}
