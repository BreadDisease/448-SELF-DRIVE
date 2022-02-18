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

// PID parameters
#define PID_CYCLE_TIME_MS 200
#define PID_DRIVE_KP 4
#define PID_DRIVE_KI 8
#define PID_DRIVE_KD 0

// Magnetometer calibration
#define MAG_X_MIN  -28.64
#define MAG_X_MAX   21.63
#define MAG_Y_MIN  -39.82
#define MAG_Y_MAX    5.91
#define MAG_DECLINATION -0.07214  // -5º 52' (Oxford, OH)

// Pin assignments
#define PIN_ENCODER_LEFT  19
#define PIN_ENCODER_RIGHT 18

// END OF CONFIGURATION
// =======================

enum ControlMode { STOCK, AUTO };
enum State { RUN, STOP };

/* =======================
 * State Variables
 * ======================= */
State state = STOP;              // Current state
TinyGPSLocation startLocation;   // Location recorded when state changes
float startHeading;              // Heading recorded on boot

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
    double setpointL = 1;
    double setpointR = 1;
    double speedL = 0;
    double speedR = 0;
    double speedS = 0;
    double actSteerPos = 0;
    double relSteerPos = 0;
    double setpointS = 0;
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
    Serial.begin(9600);
    Serial2.begin(115200);

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

    // Set up PID cycle time to be controlled manually
    PID::L.setTimeStep(1);
    PID::R.setTimeStep(1);
    PID::S.setTimeStep(1);
    PID::C.setTimeStep(1);

    // Set control mode to AUTO
    setControlMode(AUTO);

    // Initialize magnetometer
    Compass::mag.begin();

    // Wait for GPS to acquire fix
    /* while (!gps.hdop.isValid() || gps.hdop.value() > 1.5 || !gps.location.isValid()) {
        getGPSData();
    } */
    delay(2000);

    // Get start heading
    startHeading = getCompassData();

    // Fix acquired, set state to RUN
    setState(RUN);
}

void loop() {
    getGPSData();
    getCompassData();
    runPID();

    switch (state) {
        case RUN:
            PID::setpointL = 1;
            PID::setpointR = 1;

            // Differential drive
            // Adjust rear wheel differential to correct heading
            if (Compass::relHeading > 0) {
                // Turn left (right wheel is faster)
                PID::setpointL -= 1;
                PID::setpointR += 1;
            } else if (Compass::relHeading < 0) {
                // Turn right (left wheel is faster)
                PID::setpointL += 1;
                PID::setpointR -= 1;
            }
            break;
    }
}

/**
 * Runs the PID calculations at the appropriate time interval, resets the
 * encoder ticks, and updates the motor speeds.
 */
void runPID() {
    if (millis() - PID::lastCycleTime > PID_CYCLE_TIME_MS) {
        PID::actSteerPos = analogRead(A1);
        PID::L.run();
        PID::R.run();
        PID::C.run();

        PID::setpointS = PID::relSteerPos + 620;
        PID::S.run();

        noInterrupts();
        Encoder::ticksL    = 0;
        Encoder::ticksR    = 0;
        PID::lastCycleTime = millis();
        interrupts();

        Serial.println(Compass::relHeading);

        // Motor::L.setSpeed(PID::speedL);
        // Motor::R.setSpeed(PID::speedR);
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
    while (Serial2.available()) {
        gps.encode(Serial2.read());
    }
}

/**
 * Gets the current compass data from the HMC5883L and stores it in
 * Compass::heading to be used by the PID controller.
 * 
 * See: https://reprage.com/post/measuring-your-direction-with-arduino
 */
float getCompassData() {
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
    Compass::relHeading = headingDegrees - startHeading;  // TARGET HEADING
    if (Compass::relHeading > 180) {
        Compass::relHeading -= 360;
    } else if (Compass::relHeading < -180) {
        Compass::relHeading += 360;
    }

    return headingDegrees;
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
