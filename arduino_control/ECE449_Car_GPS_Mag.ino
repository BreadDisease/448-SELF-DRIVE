/*
 * Copyright (C) 2022 Matt Aaron, Taylor Owens, and Aidan Quimby.
 */
#include <AutoPID.h>
#include <CytronMotorDriver.h>
#include <TinyGPSPlus.h>
#include <Adafruit_HMC5883_U.h>

/* =======================
 * Configuration
 * ======================= */
// Target heading
#define TARGET_HEADING 0

// PID parameters
#define PID_CYCLE_TIME_MS 200
#define PID_DRIVE_KP 4
#define PID_DRIVE_KI 8
#define PID_DRIVE_KD 0.05
#define PID_STEER_KP 4
#define PID_STEER_KI 2
#define PID_STEER_KD 0

// Magnetometer calibration
#define MAG_X_MIN  -23.09
#define MAG_X_MAX   27.64
#define MAG_Y_MIN  -54.91
#define MAG_Y_MAX -101.64
#define MAG_DECLINATION -0.07214  // -5º 52' (Oxford, OH)

// Pin assignments
#define PIN_ENCODER_LEFT  20
#define PIN_ENCODER_RIGHT 21

// END OF CONFIGURATION
// =======================

enum ControlMode { STOCK, AUTO };
enum State { RUN, STOP };

/* =======================
 * State Variables
 * ======================= */
State state = STOP;              // Current state
TinyGPSLocation startLocation;   // Location recorded when state changes

/* =======================
 * Motor Drivers
 * ======================= */
namespace Motor {
    CytronMD L(PWM_DIR, 3, 2);
    CytronMD R(PWM_DIR, 5, 4);
    CytronMD S(PWM_DIR, 6, 7);
}

/* =======================
 * GPS and Compass
 * ======================= */
TinyGPSPlus gps;
namespace Compass {
    Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
    double relHeading = 0;
}

/* =======================
 * PID Variables
 * ======================= */
namespace Encoder {
    volatile double ticksL;
    volatile double ticksR;
}

namespace PID {
    unsigned long lastCycleTime = 0;
    double setpointL = 0;
    double setpointR = 0;
    double speedL = 0;
    double speedR = 0;
    double speedS = 0;

    // Left and right drive PIDs
    AutoPID L(&Encoder::ticksL,     &setpointL, &speedL,   0, 255, PID_DRIVE_KP, PID_DRIVE_KI, PID_DRIVE_KD);
    AutoPID R(&Encoder::ticksR,     &setpointR, &speedR,   0, 255, PID_DRIVE_KP, PID_DRIVE_KI, PID_DRIVE_KD);
    
    // Steering PID
    // Setpoint is set to zero because we work with relative heading
    AutoPID S(&Compass::relHeading,          0, &speedS, -75,  75, PID_STEER_KP, PID_STEER_KI, PID_STEER_KD);
}

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    Serial1.begin(115200);

    // Initialize encoders
    pinMode(PIN_ENCODER_LEFT,  INPUT);
    pinMode(PIN_ENCODER_RIGHT, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT),  encoderLeftISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), encoderRightISR, RISING);

    // Configure control relay pins
    pinMode(A10, OUTPUT);
    pinMode(A11, OUTPUT);
    pinMode(A12, OUTPUT);
    pinMode(A13, OUTPUT);
    pinMode(A14, OUTPUT);
    pinMode(A15, OUTPUT);

    // Set control mode to AUTO
    setControlMode(AUTO);

    // Initialize magnetometer
    Compass::mag.begin();

    // Wait for GPS to acquire fix
    while (!gps.hdop.isValid() || gps.hdop.value() > 1.5 || !gps.location.isValid()) {
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
            PID::setpointL = 4;
            PID::setpointR = 4;

            // Differential drive
            // Adjust rear wheel differential to correct heading
            if (Compass::relHeading > 0) {
                // Turn left (right wheel is faster)
                PID::setpointL -= 2;
                PID::setpointR += 2;
            } else if (Compass::relHeading < 0) {
                // Turn right (left wheel is faster)
                PID::setpointL += 2;
                PID::setpointR -= 2;
            }
            break;
        case STOP:
            // Stop motors
            PID::setpointL = 0;
            PID::setpointR = 0;
            break;
    }
}

/**
 * Runs the PID calculations at the appropriate time interval, resets the
 * encoder ticks, and updates the motor speeds.
 */
void runPID() {
    if (millis() - PID::lastCycleTime > PID_CYCLE_TIME_MS) {
        PID::L.run();
        PID::R.run();
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
 * Simple helper method to store the current location before changing states.
 *
 * \param[in] state The state to change to.
 */
void setState(State newState) {
    startLocation = gps.location;
    state         = newState;
}

/**
 * Gets the current GPS data from Serial1 (pins 18 and 19).
 */
void getGPSData() {
    while (Serial1.available()) {
        gps.encode(Serial1.read());
    }
}

/**
 * Gets the current compass data from the HMC5883L and stores it in
 * Compass::heading to be used by the PID controller.
 * 
 * See: https://reprage.com/post/measuring-your-direction-with-arduino
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

    // For PID steering, we need to convert this absolute heading to a relative
    // heading in the range of -180 to 180, such that the setpoint can be zero.
    Compass::relHeading = headingDegrees - TARGET_HEADING;
    if (Compass::relHeading > 180) {
        Compass::relHeading -= 360;
    } else if (Compass::relHeading < -180) {
        Compass::relHeading += 360;
    }
}

/**
 * Helper method to set the control mode. Sets all of the control relay pins to
 * either high or low, depending on the desired state.
 */
void setControlMode(ControlMode mode) {
    digitalWrite(A10, (mode == AUTO) ? LOW : HIGH);
    digitalWrite(A11, (mode == AUTO) ? LOW : HIGH);
    digitalWrite(A12, (mode == AUTO) ? LOW : HIGH);
    digitalWrite(A13, (mode == AUTO) ? LOW : HIGH);
    digitalWrite(A14, (mode == AUTO) ? LOW : HIGH);
    digitalWrite(A15, (mode == AUTO) ? LOW : HIGH);
}

/**
 * Interrupt service routine for the left encoder.
 */
void encoderLeftISR() {
    Encoder::ticksL += 1;
}

/**
 * Interrupt service routine for the right encoder.
 */
void encoderRightISR() {
    Encoder::ticksR += 1;
}
