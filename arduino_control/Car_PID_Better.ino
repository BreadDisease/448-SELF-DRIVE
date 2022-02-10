#include <AutoPID.h>
#include <CytronMotorDriver.h>

// Initialize motor driver library
CytronMD motorL(PWM_DIR, 3, 2);
CytronMD motorR(PWM_DIR, 5, 4);

const int HALL_PIN_L = 18;
const int HALL_PIN_R = 19;

#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 1
#define KI 1
#define KD 1

volatile double countL, countR, offsetL, offsetR;
double speedL, speedR;
double setPoint = 2;

unsigned long lastResetMillis = 0;

AutoPID PIDL(&countL, &setPoint, &speedL, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID PIDR(&countR, &setPoint, &speedR, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void setup() {
  Serial.begin(9600);

  pinMode(HALL_PIN_L, INPUT);
  pinMode(HALL_PIN_R, INPUT);

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

  attachInterrupt(digitalPinToInterrupt(HALL_PIN_L), leftWhlCnt, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_R), rightWhlCnt, RISING);

  countL = 0;
  countR = 0;
  offsetL = 0;
  offsetR = 0;

  delay(100);

  PIDL.setTimeStep(100);
  PIDR.setTimeStep(100);

  lastResetMillis = millis();
}

void loop() {
  delay(500);
  PIDL.run();
  PIDR.run();
  countL = 0;
  countR = 0;
  motorL.setSpeed(speedL);
  motorR.setSpeed(speedR);
}

void leftWhlCnt() {
  countL++;
}

void rightWhlCnt() {
  countR++;
}
