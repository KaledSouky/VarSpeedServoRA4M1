/*
  SweepTwoServos
  By Philip van Allen <philvanallen.com> for the 
  VarSpeedServo.h library (October 2013)
  Adapted by Kaled Souky <kaledsouky2@gmail.com> for the 
  VarSpeedServoRA4M1.h library (December 2024)
  This example code is in the public domain.
  
  -------------------------------------------------------
  
  Sweeps two servos synchronously, from 0 to 180 degrees 
  at fast speed and back from 180 to 0 degrees at slow 
  speed, in an infinite loop.

  The write() function now includes the "speed" and 
  "wait" parameters, which can optionally be used in 
  conjunction with the "position" parameter to control 
  servo movement.

  The "wait" parameter is used to wait for the servo to
  complete its movement. It is also available as a 
  standalone wait() function.

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
 
/* Create servo objects to control servos, a maximum of twelve servo 
   objects can be created
*/
VarSpeedServoRA4M1 myservo1;  
VarSpeedServoRA4M1 myservo2;

const int servoPin1 = 9;  // the digital pin used for the first servo
const int servoPin2 = 10;  // the digital pin used for the second servo
 
void setup() { 
  myservo1.attach(servoPin1);  // attaches the servo on pin 9 to the servo object
  myservo1.write(0,127,false);  // set the servo to its initial position at fast speed, run in background
  myservo2.attach(servoPin2);  // attaches the servo on pin 10 to the servo object
  myservo2.write(0,127,true);  // set the servo to its initial position at fast speed, wait for completion
} 

void loop() {
  // write(degrees 0-180, speed 1-255, wait to complete true-false)
  /* On the RC servos tested, speed differences above 127 
     cannot be noticed due to the mechanical limitations of 
     the servo. However, feel free to experiment with the 
     speed value that best suits the response of your servo 
     based on its mechanical and electronic characteristics 
  */
  myservo1.write(0,127,false);  // move the servo to 0, fast speed, run in background
  myservo2.write(0,127,true);  // move the servo to 0, fast speed, wait until done
  
  myservo1.write(180,30,false);  // move the servo to 180, slow speed, run in background
  myservo2.write(180,30,true);  // move the servo to 180, slow speed, wait until done
} 
