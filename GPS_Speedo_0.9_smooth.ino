/*
  GPS speedometer.
  (c) 1st November 2017 A.G.Doswell  Web Page http://andydoz.blogspot.co.uk

  Available to download from https://github.com/andydoswell/GPS-Speedo

  License: The MIT License (See full license at the bottom of this file)

  The sketch uses a U-Blox 6M GPS satellite module connected to the hardware Serial interface,
  a 128x32 SDD1306 OLED display module connected as an I2C device to pins A4 (SDA)& A5 (SCL) and a Swtech stepper motor connected to pins 4 -7. The arduino used is a Nano with 5v ATMEGA328P.

  The OzOled library is not quite "right" for this application, but works, and is lightweight and fast enough. Thanks Oscar!
  It is available here : http://blog.oscarliang.net/arduino-oled-display-library/
  TinyGPSPlus is available here :https://github.com/mikalhart/TinyGPSPlus
  Switec X25 stepper motor library is availble here: https://github.com/clearwater/SwitecX25

*/

#include <Wire.h>
#include <OzOLED.h>
#include <TinyGPS++.h>
#include <EEPROM.h>
#include <SwitecX25.h>
int MPH;                         // Speed in mph
unsigned int motorPosition;
SwitecX25 motor1(945, 4, 5, 6, 7); // set up stepper motor, 945 steps, pins 4,5,6 & 7
TinyGPSPlus gps;                   // Feed gps data to TinySGPSPlus
int milesUnit = 0;
int milesTen = 0;
int milesHundred = 0;
int milesThousand = 0;
int milesTenThousand = 0;
char milesUnitA [2];
char milesTenA [2];
char  milesHundredA [2];
char milesThousandA [2];
char milesTenThousandA [2];
double oldLat;
double oldLong;
float distanceMeters;
boolean fixFlag = false;
const int maxSpeed = 120;
const int maxStep = 784;
int delayCounter = 0 ;
int oldSpeed;
int acceleration;
long currentMotorPosition;
long motorDifference ;
float currentDistance;


void setup()   {
  pinMode(8, OUTPUT); // output to power supply over-ride, high gets power from Permanent 12V
  pinMode(9, INPUT); // input from IGN signal conditioning.
  digitalWrite(8, HIGH); // set power to main.
  OzOled.init();                   // initialze SDD1306 OLED display
  OzOled.sendCommand(0x8d);        // Set displays inbuilt inverter on
  OzOled.sendCommand(0x14);
  OzOled.setBrightness(0xFF);      // ... and brightness to max
  OzOled.clearDisplay();           // Clear the screen
  OzOled.setNormalDisplay();       // Set display to Normal mode
  motor1.zero(); // zero the stepper motor
  motor1.setPosition (maxStep); // move to the other end
  motor1.updateBlocking();
  motor1.zero(); // and back to zero
  updateDisplay();
  delay (6000); // allow the u-blox receiver to come up
  Serial.begin(9600);// start the comms with the GPS Rx
  // send Serial to update u-blox rate to 200mS
  Serial.write(0xB5);
  Serial.write(0x62);
  Serial.write(0x06);
  Serial.write(0x08);
  Serial.write(0x06);
  Serial.write(0x00);
  Serial.write(0xC8);
  Serial.write(0x00);
  Serial.write(0x01);
  Serial.write(0x00);
  Serial.write(0x01);
  Serial.write(0x00);
  Serial.write(0xDE);
  Serial.write(0x6A);
  Serial.write(0xB5);
  Serial.write(0x62);
  Serial.write(0x06);
  Serial.write(0x08);
  Serial.write(0x00);
  Serial.write(0x00);
  Serial.write(0x0E);
  Serial.write(0x30);
  delay (100);
  Serial.flush();
  // set 57,600 baud on u-blox
  Serial.write(0xB5);
  Serial.write(0x62);
  Serial.write(0x06);
  Serial.write(0x00);
  Serial.write(0x14);
  Serial.write(0x00);
  Serial.write(0x01);
  Serial.write(0x00);
  Serial.write(0x00);
  Serial.write(0x00);
  Serial.write(0xD0);
  Serial.write(0x08);
  Serial.write(0x00);
  Serial.write(0x00);
  Serial.write(0x00);
  Serial.write(0xE1);
  Serial.write(0x00);
  Serial.write(0x00);
  Serial.write(0x07);
  Serial.write(0x00);
  Serial.write(0x02);
  Serial.write(0x00);
  Serial.write(0x00);
  Serial.write(0x00);
  Serial.write(0x00);
  Serial.write(0x00);
  Serial.write(0xDD);
  Serial.write(0xC3);
  Serial.write(0xB5);
  Serial.write(0x62);
  Serial.write(0x06);
  Serial.write(0x00);
  Serial.write(0x01);
  Serial.write(0x00);
  Serial.write(0x01);
  Serial.write(0x08);
  Serial.write(0x22);
  delay (100);
  Serial.end();// stop Serial coms at 9,600 baud
  delay (100);
  Serial.begin (57600); // start Serial coms at 57,600 baud.
  // Get mileage from EEPROM
  getMileage ();
  updateDisplay ();

  //setMileage (); // writes a new mileage to the EEPROM. Run this once, and then comment it out, and reload the sketch. You can set the mileage required in the setMileage function.
}

void loop() {
  while (Serial.available()) {//While GPS message received,
    if (gps.encode(Serial.read())); // if the GPS messaged received is ready
    {
      processGPS ();
      break;
    }
  }

  if (MPH >= 3) {
    motorPosition = map (MPH, 0 , maxSpeed , 1, maxStep);
    motor1.setPosition(motorPosition);
  }
  else {
    motorPosition = 1; // don't go quite to zero... prevents potential issue damaging the motor.
    motor1.setPosition(motorPosition);
    motor1.updateBlocking ();
    currentMotorPosition = motorPosition;

  }

  if (delayCounter <= 0) {
    updateMotor ();
  }
  delayCounter --;
  checkPower ();
}

void checkPower () {
  if (!(digitalRead(9))) { // if Pin 9 is low, then write mileage to the EEPROM and shutdown
    delay (30000);
    if (!(digitalRead(9))) { // check power again, just to be sure (all this paranoia is to prevent EEPROM corruption)
      updateEEPROM ();
      delay (1500);
      digitalWrite (8, LOW);// switch off power
    }
  }
}

void processGPS () {
  if (gps.location.isValid ()) {
    MPH = gps.speed.mph();
    if (!fixFlag) {
      oldLat = gps.location.lat();
      oldLong = gps.location.lng();
      getMileage (); // re-read the EEPROM, just in case it was corrupted during cranking.
      fixFlag = true;

    }
    else {
      if (MPH >= 3) {
        currentDistance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), oldLat, oldLong);
        distanceMeters += currentDistance;
        oldLat = gps.location.lat();
        oldLong = gps.location.lng();
      }

    }
    if (distanceMeters >= 1609.34) { // 1609.34m in a mile
      distanceMeters -= 1609.34;
      if (distanceMeters < 10) { // this prevents a rare error which screws the mileage.
        incrementMileage ();
      }
    }
  }
}

void updateDisplay() {               // Display the data
  itoa (milesTenThousand, milesTenThousandA, 10);
  itoa (milesThousand, milesThousandA, 10);
  itoa (milesHundred, milesHundredA, 10);
  itoa (milesTen, milesTenA, 10);
  itoa (milesUnit, milesUnitA, 10);

  OzOled.printBigNumber (milesTenThousandA, 0, 1);
  OzOled.printBigNumber (milesThousandA, 3, 1);
  OzOled.printBigNumber (milesHundredA, 6, 1);
  OzOled.printBigNumber (milesTenA, 9, 1);
  OzOled.printBigNumber (milesUnitA, 12, 1);
  //OzOled.printString("THIS WAY UP!", 3, 0); // uncomment this when glueing the display!

}

void incrementMileage () { // Increment the mileage.
  milesUnit ++;
  if (milesUnit == 10) {
    milesTen++;
    milesUnit = 0;
  }
  if (milesTen == 10) {
    milesHundred++;
    milesTen = 0;
  }
  if (milesHundred == 10) {
    milesThousand++;
    milesHundred = 0;
  }
  if (milesThousand == 10) {
    milesTenThousand++;
    milesThousand = 0;
  }
  if (milesTenThousand == 10) { // gone round the clock 'guv!
    milesUnit = 0;
    milesTen = 0;
    milesHundred = 0;
    milesThousand = 0;
    milesTenThousand = 0;
  }
  updateDisplay();
}

void updateEEPROM() {
  EEPROM.update (0, milesUnit);
  EEPROM.update (1, milesTen);
  EEPROM.update (2, milesHundred);
  EEPROM.update (3, milesThousand);
  EEPROM.update (4, milesTenThousand);
  EEPROM.put (5, distanceMeters);

}

void setMileage () {
  milesUnit = 6;
  milesTen = 1;
  milesHundred = 9;
  milesThousand = 3;
  milesTenThousand = 6;
  distanceMeters = 0;
  updateEEPROM ();
}

void getMileage () {
  milesUnit = EEPROM.read (0);
  milesTen = EEPROM.read (1);
  milesHundred = EEPROM.read (2);
  milesThousand = EEPROM.read (3);
  milesTenThousand = EEPROM.read (4);
  EEPROM.get (5, distanceMeters);
}

void updateMotor () {

  if (currentMotorPosition < motorPosition) {
    currentMotorPosition ++;
  }

  if (currentMotorPosition > motorPosition) {
    currentMotorPosition --;
  }
  motorDifference = motorPosition - currentMotorPosition;
  motorDifference = abs(motorDifference);
  if (motorDifference > 40) {
    delayCounter = 0;
  }
 /* if (motorDifference <= 50) {
    delayCounter = 100;
  }*/
  if (motorDifference <= 40) {
    delayCounter = 40;
  }
  if (motorDifference <= 30) {
    delayCounter = 133;
  }
  if (motorDifference <= 20) {
    delayCounter = 300;
  }
  if (motorDifference <= 10) {
    delayCounter = 600;
  }
  if (motorDifference <= 6) {
    delayCounter = 1000;
  }
  motor1.update ();
}
/*
   Copyright (c) 2017 Andrew Doswell

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permiSerialion notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESerial FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHOR(S) OR COPYRIGHT HOLDER(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
*/
