#include <AutoPID.h>

//pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 6
#define KI 1
#define KD 4
 
 // PWM Setting
 double RSPD1 = 140;        //Right Wheel PWM.  Change this value so your car will go roughly straight
 double LSPD1 = 140;        //Left Wheel PWM
 double RSPD2 = 140;        //Right Wheel PWM
 double LSPD2 = 140;        //Left Wheel PWM

const int LWhFwdPin = 13;   // Connect to L298H
const int LWhBwdPin = 12;
const int LWhPWMPin = 11;

const int RWhFwdPin = 8;
const int RWhBwdPin = 7;
const int RWhPWMPin = 6; 

volatile double cntrL, cntrR, diff, setPoint;

AutoPID myPID(&diff, &setPoint, &LSPD1, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(LWhFwdPin,OUTPUT);
  pinMode(LWhBwdPin,OUTPUT);
  pinMode(LWhPWMPin,OUTPUT);
  pinMode(RWhFwdPin,OUTPUT);
  pinMode(RWhBwdPin,OUTPUT);
  pinMode(RWhPWMPin,OUTPUT); 

  digitalWrite(LWhFwdPin,HIGH);
  digitalWrite(LWhBwdPin,LOW);
  
  digitalWrite(RWhFwdPin,HIGH);
  digitalWrite(RWhBwdPin,LOW);

  attachInterrupt(digitalPinToInterrupt(2), leftWhlCnt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), rightWhlCnt, CHANGE);

  cntrR = 0;
  cntrL = 0;
  diff = 0;
  setPoint = 0;

  analogWrite(RWhPWMPin,RSPD1);     //turn on wheels 
  analogWrite(LWhPWMPin,LSPD1);

  myPID.setTimeStep(100);
}

void loop() {
  Serial.println("Counters:");
  Serial.println(cntrR);
  Serial.println(cntrL);
  Serial.println();
  Serial.println("PWM:");
  Serial.println(RSPD1);
  Serial.println(LSPD1);
  Serial.println();
  diff = cntrR - cntrL;
  myPID.run();
  analogWrite(LWhPWMPin, LSPD1);
}
void leftWhlCnt()
{
  cntrL++;
}

void rightWhlCnt()  // Complete this ISR
{
  cntrR++;
}
