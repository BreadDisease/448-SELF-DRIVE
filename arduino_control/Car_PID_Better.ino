// Copyright 2022 Matt Aaron
#include <AutoPID.h>
#include <CytronMotorDriver.h>

// Pins
#define PIN_ENCODER_L 18
#define PIN_ENCODER_R 19

// Initialize motor driver library
CytronMD motorL(PWM_DIR, 3, 2);
CytronMD motorR(PWM_DIR, 5, 4);
CytronMD motorS(PWM_DIR, 6, 7);

// PID stuff
#define PID_CYCLE_TIME 100  // Milliseconds
#define PID_OUTPUT_MIN 0
#define PID_OUTPUT_MAX 255
#define PID_KP 1
#define PID_KI 1
#define PID_KD 1
unsigned long tickCycleStart = 0;
volatile double ticksL, ticksR;
double setpointL, setpointR;
double speedL, speedR;

AutoPID pidL(&ticksL, &setpointL, &speedL, PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_KP, PID_KI, PID_KD);
AutoPID pidR(&ticksR, &setpointR, &speedR, PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_KP, PID_KI, PID_KD);

void setup() {
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

  setpointL = 0.5;  // 5 ticks per second (mult. by 0.1s)
  setpointR = 0.5;
  // Time step is handled manually in loop(), so set to minimum
  pidL.setTimeStep(1);
  pidR.setTimeStep(1);
  tickCycleStart = millis();
}

void loop() {
  if (millis() - tickCycleStart > PID_CYCLE_TIME) {
    pidL.run();
    pidR.run();

    // Disable interrupts and reset ticks
    noInterrupts();
    ticksL = 0;
    ticksR = 0;
    tickCycleStart = millis();
    interrupts();
    
    motorL.setSpeed(speedL);
    motorR.setSpeed(speedR);
  }
}

void tickL() {
  ticksL++;
}

void tickR() {
  ticksR++;
}

