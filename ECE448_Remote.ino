/*
   Aidan Quimby
   ECE 448
   Self Driving Car Remote
   10/26/2021
*/

// Library for RF tranmission
#include <ELECHOUSE_CC1101.h>

// Size of wireless tranmission in bytes
#define size 3

// Transmission buffer
byte TX_buffer[size] = {0};

// Joystick variables
int VRx = A0;
int VRy = A1;
int SW = 3;

// Joystick positions that are transmitted wirelessly
int xPosition = 0;
int yPosition = 0;
int SW_state = 0;

// Method to print transmission buffer to Serial Monitor
void printData() {
  Serial.print("X position: ");
  Serial.println(TX_buffer[0]);
  Serial.print("Y Position: ");
  Serial.println(TX_buffer[1]);
  Serial.print("Button State: ");
  Serial.println(TX_buffer[2]);
}

void setup() {
  Serial.begin(9600);
  pinMode(VRx, INPUT);
  pinMode(VRy, INPUT);
  pinMode(SW, INPUT_PULLUP);
  ELECHOUSE_cc1101.Init();
}

void loop() {
  // Math to convert joystick range from 0-1023 to 0-255
  xPosition = analogRead(VRx) / 4;
  yPosition = analogRead(VRy) / 4;
  SW_state = digitalRead(SW);

  // Entering joystick positions into transmission buffer
  TX_buffer[0] = xPosition;
  TX_buffer[1] = yPosition;
  TX_buffer[2] = SW_state;
  
  printData();

  ELECHOUSE_cc1101.SendData(TX_buffer, size);
  delay(250);
}
