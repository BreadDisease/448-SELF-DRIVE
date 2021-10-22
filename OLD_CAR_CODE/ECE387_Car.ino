/* Run Wheels, Servo, and remote
*/
#include <Wire.h>
#include <ELECHOUSE_CC1101.h>
#include <Servo.h>
                          // Wire colors to keep track of which wire is in which pin
const int LWhFwdPin = 4;  // blue
const int LWhBwdPin = 3;  // purple
const int LWhPWMPin = 5;  // green

const int RWhFwdPin = 8;  // grey
const int RWhBwdPin = 7;  // white
const int RWhPWMPin = 6;  // yellow

// Reciever variables
byte RX_buffer[3] = {0};
byte size, i, flag;
int xPos, yPos, btn;

// Servo object for the catapult
Servo catapult;
// Variable for servo position
int pos = 180;

int RSPD1 = 170;        //Right Wheel PWM
int LSPD1 = 150;        //Left Wheel PWM
int RSPD2 = 170;        //Right Wheel PWM
int LSPD2 = 150;        //Left Wheel PWM

void stopMotors() {
  analogWrite(LWhPWMPin, 0);      // Stop left wheel
  analogWrite(RWhPWMPin, 0);      // Stop right wheel
  digitalWrite(LWhFwdPin, LOW);
  digitalWrite(LWhBwdPin, LOW);
  digitalWrite(RWhFwdPin, LOW);
  digitalWrite(RWhBwdPin, LOW);
}

void drive() {
  stopMotors();
  if (xPos >= 135) {
    digitalWrite(LWhFwdPin, HIGH);
    digitalWrite(RWhFwdPin, HIGH);
    digitalWrite(RWhPWMPin, xPos);
    digitalWrite(LWhPWMPin, xPos);
    //Serial.println(xPos);
  } else if (xPos <= 125) {
    digitalWrite(LWhBwdPin, HIGH);
    digitalWrite(RWhBwdPin, HIGH);
    analogWrite(RWhPWMPin, (255 - xPos));
    analogWrite(LWhPWMPin, (255 - xPos));
    //Serial.println(255 - xPos);
  }

  if (yPos >= 140) {
    digitalWrite(LWhFwdPin, HIGH);
    analogWrite(LWhPWMPin, 255);
  } else if (yPos <= 110) {
    digitalWrite(RWhFwdPin, HIGH);
    analogWrite(RWhPWMPin, 255);
  }
}

void throwCatapult() {
  // Stop the car from moving so you don't lose control
   stopMotors();
  // launch
  pos = 70;
  catapult.write(pos);
  // Return catapult to original position
  for (pos = 70; pos <= 180; pos += 5) { // goes from -90 degrees to 0 degrees
    catapult.write(pos); 
    delay(20);
  }
}

void getData() {
  catapult.write(pos);
  size = ELECHOUSE_cc1101.ReceiveData(RX_buffer);
  xPos = RX_buffer[0];
  yPos = RX_buffer[1];
  btn = RX_buffer[2];
  if (btn == 0) {
    throwCatapult();
  } else {
    drive();
  }
  Serial.println(xPos);
  Serial.println(yPos);
  Serial.println(btn);
  ELECHOUSE_cc1101.SetReceive();
}


void setup() {
  Serial.begin(9600);

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.SetReceive();

  catapult.attach(A5);  // Servo is attached to pin 1 - 1 is sometimes finicky

  pinMode(LWhFwdPin, OUTPUT);
  pinMode(LWhBwdPin, OUTPUT);
  pinMode(LWhPWMPin, OUTPUT);
  pinMode(RWhFwdPin, OUTPUT);
  pinMode(RWhBwdPin, OUTPUT);
  pinMode(RWhPWMPin, OUTPUT);

  digitalWrite(LWhFwdPin, LOW);
  digitalWrite(LWhBwdPin, LOW);
  digitalWrite(LWhPWMPin, LOW);

  digitalWrite(RWhFwdPin, LOW);
  digitalWrite(RWhBwdPin, LOW);
  digitalWrite(RWhPWMPin, LOW);
}

void loop() {
  if (ELECHOUSE_cc1101.CheckReceiveFlag()) {
    getData();
  }
}
