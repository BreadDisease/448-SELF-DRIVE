/*
   Aidan Quimby
   Taylor Owens
   Matt Aaron
   ECE 448
   Self Driving Car Driving Program with Raspberry Pi Control
   11/05/2021
*/

#include <avr/sleep.h>
#include <cc1100_arduino.h>
#include <EnableInterrupt.h>
#include "CytronMotorDriver.h"

// Configure the motor driver.
CytronMD motorL(PWM_DIR, 3, 2);  // PWM 1 = Pin 3, DIR 1 = Pin 2.
CytronMD motorR(PWM_DIR, 5, 4); // PWM 2 = Pin 5, DIR 2 = Pin 4.
CytronMD motorS(PWM_DIR, 6, 7); // PWM 3 = Pin 6, DIR 3 = Pin 7.

// Pin to switch relay state
int relayPin = A0;

// Motor speed variables
int xSpeed, ySpeed;
int change = 30;

/*
 * Pi Variables
 * dir is 0 for idle, 1 for forward, 2 for backwards
 * steer is 0 for straight, 1 for hard left, 2 for hard right, 3 for soft left, 4 for soft right
 */
int dir, steer, speedVal, sw;

// Boolean to state switch between stock remote control and our remote control
bool state = true;

// Global CC1100 variables
uint8_t Tx_fifo[FIFOBUFFER], Rx_fifo[FIFOBUFFER];
uint8_t My_addr, Tx_addr, Rx_addr, Pktlen, pktlen, Lqi, Rssi;
uint8_t rx_addr, sender, lqi;
int8_t rssi_dbm;
volatile uint8_t cc1101_packet_available;

// CC1101 Constructor
CC1100 cc1100;

void setup()
{
  pinMode(relayPin, OUTPUT);

  // init serial Port for debugging
  Serial.begin(115200); Serial.println();

  // init CC1101 RF-module and get My_address from EEPROM
  cc1100.begin(My_addr);                   //inits RF module with main default settings

  cc1100.sidle();                          //set to ILDE first

  cc1100.set_mode(0x04);                   //set modulation mode 1 = GFSK_1_2_kb; 2 = GFSK_38_4_kb; 3 = GFSK_100_kb; 4 = MSK_250_kb; 5 = MSK_500_kb; 6 = OOK_4_8_kb
  cc1100.set_ISM(0x02);                    //set ISM Band 1=315MHz; 2=433MHz; 3=868MHz; 4=915MHz
  cc1100.set_channel(0x01);                //set channel
  cc1100.set_output_power_level(0);        //set PA level in dbm
  cc1100.set_myaddr(0x03);                 //set my own address

  //cc1100.spi_write_register(IOCFG0, 0x24); //set module in sync mode detection mode
  cc1100.spi_write_register(IOCFG2, 0x06);

  cc1100.show_main_settings();             //shows setting debug messages to UART
  cc1100.show_register_settings();         //shows current CC1101 register values

  cc1100.receive();                        //set to RECEIVE mode

  // init interrrupt function for available packet
  enableInterrupt(GDO2, rf_available_int, RISING);

  Serial.println(F("Finished Setup."));   //welcome message
}

void loop() {
  // If packet is received, read data in
  if (cc1101_packet_available == TRUE) {
    dir = Rx_fifo[3];
    steer = Rx_fifo[4];
    speedVal = Rx_fifo[5];
    sw = Rx_fifo[6];
    drive();

    cc1101_packet_available = FALSE;
  }
}

// Check incoming RF packet
void rf_available_int(void) {
  disableInterrupt(GDO2);

  if (cc1100.packet_available() == TRUE) {
    if (cc1100.get_payload(Rx_fifo, pktlen, rx_addr, sender, rssi_dbm, lqi) == TRUE) {
      cc1101_packet_available = TRUE;
    } else {
      cc1101_packet_available = FALSE;
    }
  }
  enableInterrupt(GDO2, rf_available_int, RISING);
}

void changeXSpeed() {
  if (dir == 1 && xSpeed <= speedVal - change) {
    xSpeed += change;
  } else if (dir == 2 && xSpeed >= speedVal + change) {
    xSpeed -= change;
  } else if (dir == 0) {
    slowDownX();
  }
}

void changeYSpeed() {
  if (steer == 1 || steer == 2) {
    tankTurn(steer);
  } else if (steer == 3) {
    ySpeed += change;
    motorL.setSpeed(xSpeed);   // Drive left motor at x value
    motorR.setSpeed(xSpeed);   // Drive right motor at x value
    motorS.setSpeed(ySpeed);   // Drive steering motor at y value
  } else if (steer == 4) {
    ySpeed -= change;
    motorL.setSpeed(xSpeed);   // Drive left motor at x value
    motorR.setSpeed(xSpeed);   // Drive right motor at x value
    motorS.setSpeed(ySpeed);   // Drive steering motor at y value
  } else if (steer == 0) {
    slowDownY();
  }
}

void slowDownX() {
  if (xSpeed > 30) {
    xSpeed -= change;
  } else if (xSpeed < -30) {
    xSpeed += change;
  } else {
    xSpeed = 0;
  }
}

void slowDownY() {
  if (ySpeed > 30) {
    ySpeed -= change;
  } else if (ySpeed < -30) {
    ySpeed += change;
  } else {
    ySpeed = 0;
  }
}

void tankTurn(int turnDir) {
  if (turnDir == 1) {
    motorL.setSpeed(-change * 2);   // Drive left motor at x value
    motorR.setSpeed(change * 2);   // Drive right motor at x value
  } else if (turnDir == 2) {
    motorL.setSpeed(change * 2);   // Drive left motor at x value
    motorR.setSpeed(-change * 2);   // Drive right motor at x value
  }
}

void drive() {
  if (sw == 0) {
    state = !state;
  }
  if (state) {
    digitalWrite(relayPin, LOW);
    Serial.println("Driving...");
    changeXSpeed();
    changeYSpeed();
  } else {
    xSpeed = 0;
    ySpeed = 0;
    digitalWrite(relayPin, HIGH);
  }
}
