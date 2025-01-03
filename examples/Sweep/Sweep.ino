/*
  Sweep
  By BARRAGAN <http://barraganstudio.com>
  Modified on 8 Nov 2013 by Scott Fitzgerald 
  https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
  Adapted by Kaled Souky <kaledsouky2@gmail.com> for the 
  VarSpeedServoRA4M1.h library (December 2024)
  This example code is in the public domain.
 
  -------------------------------------------------------
  
  Sweeps a servo motor from 0 to 180 degrees at fast 
  speed and back from 180 to 0 degrees at slow speed, 
  in an infinite loop.

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
 
VarSpeedServoRA4M1 myservo;  // create servo object to control a servo 
// A maximum of twelve servo objects can be created

const int servoPin = 9;  // the digital pin used for the servo

void setup() { 
  myservo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  myservo.write(0,127,true);  // set the servo to its initial position at fast speed, wait for completion
} 

void loop() {
  // write(degrees 0-180, speed 1-255, wait to complete true-false)
  /* On the RC servos tested, speed differences above 127 
     cannot be noticed due to the mechanical limitations of 
     the servo. However, feel free to experiment with the 
     speed value that best suits the response of your servo 
     based on its mechanical and electronic characteristics 
  */
  myservo.write(180,127,true);  // move the servo to 180, fast speed, wait until done
  myservo.write(0,30,true);  // move the servo to 0, slow speed, wait until done
} 
