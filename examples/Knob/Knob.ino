/*
  Knob
  By Michal Rinott 
  http://people.interaction-ivrea.it/m.rinott 
  Modified on 8 Nov 2013 by Scott Fitzgerald 
  http://www.arduino.cc/en/Tutorial/Knob
  Adapted by Kaled Souky <kaledsouky2@gmail.com> for the 
  VarSpeedServoRA4M1.h library (December 2024)
  This example code is in the public domain.
   
  -------------------------------------------------------

  Controlling a servo position using a potentiometer 
  (variable resistor).
  
  -------------------------------------------------------

  WARNING: "External devices with a high current draw 
  (e.g., servo motors) should never be powered via the 5V 
  pin. It is mainly intended for devices drawing lower 
  current such as sensor modules."
 
  For more information about the Arduino UNO R4 Minima 
  and Arduino UNO R4 WiFi boards, please check the 
  following links from the official Arduino site:

  * https://docs.arduino.cc/tutorials/uno-r4-minima/cheat-sheet/
  * https://docs.arduino.cc/tutorials/uno-r4-wifi/cheat-sheet/

  LEARN: How to connect, power, and control servomotors 
  with your Arduino board, in the following link from the 
  official Arduino site:
 
  * https://docs.arduino.cc/learn/electronics/servo-motors/
*/

#include <VarSpeedServoRA4M1.h> 
 
VarSpeedServoRA4M1 myservo;  // create servo object to control a servo 
// A maximum of twelve servo objects can be created

const int potpin = A0;  // analog pin used to connect the potentiometer
const int servoPin = 9;  // the digital pin used for the servo

int val;  // variable to read the value from the analog pin

void setup() {
  myservo.attach(servoPin);  // attaches the servo on pin 9 to the Servo object
}

void loop() {
  val = analogRead(potpin);  // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 0, 180);  // scale it for use with the servo (value between 0 and 180)
  myservo.write(val);  // sets the servo position according to the scaled value
  delay(15);  // waits for the servo to get there
} 
