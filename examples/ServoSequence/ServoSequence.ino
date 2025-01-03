/*
  ServoSequence
  By Philip van Allen <philvanallen.com> for the 
  VarSpeedServo.h library (October 2013) 
  Adapted by Kaled Souky <kaledsouky2@gmail.com> for the 
  VarSpeedServoRA4M1.h library (December 2024)
  This example code is in the public domain.
  
  -------------------------------------------------------

  Reads an analog input, and plays different servo 
  sequences depending on the analog value.

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

VarSpeedServoRA4M1 myservo1;  // create servo object to control a servo 
// A maximum of twelve servo objects can be created

const int servoPin1 = 9;  // the digital pin used for the servo

/* Sequences are defined as an array of points, in the sequence each point 
   has a position from 0 - 180, and a speed to get to that position 
*/
/* On the RC servos tested, speed differences above 127 
   cannot be noticed due to the mechanical limitations of 
   the servo. However, feel free to experiment with the 
   speed value that best suits the response of your servo 
   based on its mechanical and electronic characteristics 
*/
servoSequencePoint slowHalf[] = {{100,20},{20,20},{60,50}};  // go to position 100 at speed 20, position 20 at speed 20, position 60 at speed 50
servoSequencePoint twitchy[] = {{0,127},{180,40},{90,100},{120,60}};  // go to position 0 at speed 127, position 180 at speed 40, position 90 at speed 100, position 120 at speed 60 

const int analogPin = A0;

// The setup routine runs once when you press reset:
void setup() {
  myservo1.attach(servoPin1);
}

// The loop routine runs over and over again forever:
void loop() {
  // Read the input on analog pin 0:
  int sensorValue = analogRead(analogPin);
  if (sensorValue > 200) {
    myservo1.sequencePlay(slowHalf, 3);  // play sequence "slowHalf" that has 3 positions, loop and start at first position
  } else {
    myservo1.sequencePlay(twitchy, 4, true, 2);  // play sequence "twitchy" that has 4 positions, loop and start at third position
  }
  delay(2);  // delay in between reads for analogin stability
}

