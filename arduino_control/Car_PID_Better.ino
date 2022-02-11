// Copyright 2022 Matt Aaron
#include <AutoPID.h>
#include <CytronMotorDriver.h>
#include <TinyGPSPlus.h>

// Pins
#define PIN_ENCODER_L 20
#define PIN_ENCODER_R 21

// Initialize GPS library and compass
TinyGPSPlus gps;
// For test sketch
TinyGPSLocation startLocation;
bool startedDriving = false;
bool reachedDest    = false;

// Initialize motor driver library
CytronMD motorL(PWM_DIR, 3, 2);
CytronMD motorR(PWM_DIR, 5, 4);
CytronMD motorS(PWM_DIR, 6, 7);

// PID stuff
#define PID_CYCLE_TIME 200  // Milliseconds
#define PID_OUTPUT_MIN 0
#define PID_OUTPUT_MAX 255
#define PID_KP 4
#define PID_KI 8
#define PID_KD 0.05
unsigned long tickCycleStart = 0;
volatile double ticksL, ticksR;
double setpointL, setpointR;
double speedL, speedR;

AutoPID pidL(&ticksL, &setpointL, &speedL, PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_KP, PID_KI, PID_KD);
AutoPID pidR(&ticksR, &setpointR, &speedR, PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_KP, PID_KI, PID_KD);

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);

  pinMode(PIN_ENCODER_L, INPUT);
  pinMode(PIN_ENCODER_R, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L), tickL, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R), tickR, RISING);

  pinMode(A10, OUTPUT);
  digitalWrite(A10, LOW);
  pinMode(A11, OUTPUT);
  digitalWrite(A11, LOW);
  pinMode(A12, OUTPUT);
  digitalWrite(A12, LOW);
  pinMode(A13, OUTPUT);
  digitalWrite(A13, LOW);
  pinMode(A14, OUTPUT);
  digitalWrite(A14, LOW);
  pinMode(A15, OUTPUT);
  digitalWrite(A15, LOW);

  setpointL = 0;  // 5 ticks per second (mult. by 0.2s)
  setpointR = 0;
  // Time step is handled manually in loop(), so set to minimum
  pidL.setTimeStep(1);
  pidR.setTimeStep(1);
  tickCycleStart = millis();
}

void loop() {
  readGPSData();

  if (!startedDriving) {
    if (gps.hdop.isValid() && gps.hdop.hdop() <= 1.5 && gps.location.isValid()) {
      startLocation = gps.location;
      startedDriving = true;
    }
  } else {
    if (millis() - tickCycleStart > PID_CYCLE_TIME) {
      pidL.run();
      pidR.run();

      Serial.print("SPEED L: ");
      Serial.print(speedL);
      Serial.print("\tTICKS L:");
      Serial.println(ticksL);
      Serial.print("SPEED R: ");
      Serial.print(speedR);
      Serial.print("\tTICKS R:");
      Serial.println(ticksR);

      if (setpointL < 5 && setpointR < 5 && !reachedDest) {
        setpointL += 1;
        setpointR += 1;
      } else if (setpointL > 0 && setpointR > 0 && reachedDest) {
        setpointL -= 1;
        setpointR -= 1;
      }

      // Disable interrupts and reset ticks
      noInterrupts();
      ticksL = 0;
      ticksR = 0;
      tickCycleStart = millis();
      interrupts();

      motorL.setSpeed(speedL);
      motorR.setSpeed(speedR);
    }

    double distanceFromStart = TinyGPSPlus::distanceBetween(startLocation.lat(), startLocation.lng(), gps.location.lat(), gps.location.lng());
    if (distanceFromStart > 9) {
      reachedDest = true;
    }
  }
}

void readGPSData() {
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }
}

void tickL() {
  ticksL++;
}

void tickR() {
  ticksR++;
}
