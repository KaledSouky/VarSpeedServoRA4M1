/*
  SweepTwoServosLegacy
  By Kaled Souky <kaledsouky2@gmail.com> for the 
  VarSpeedServoRA4M1.h library (December 2024)
  This example code is in the public domain.
  
  -------------------------------------------------------

  Sweeps two servos synchronously, from 0 to 180 degrees 
  at fast speed and back from 180 to 0 degrees at slow 
  speed, in an infinite loop.

  The slowmove() function only handles the "position" 
  and "velocity" parameters. In this case, the wait time 
  must be set (using, for example, the delay(), millis(),
  micros(), or delayMicroseconds() functions) to ensure 
  the servo completes the position and maintains it for 
  the desired time. The slowmove() function does not 
  incorporate the "wait" variable.

  This example demonstrates the use of the delay() 
  function as a timeout with slowmove(). The exact delay 
  value may require some experimentation (usually in the 
  millisecond range) to ensure that the servo has enough 
  time to reach its target position and hold it for as 
  long as necessary, until receiving the next instruction.

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
  myservo2.attach(servoPin2);  // attaches the servo on pin 10 to the servo object
 
  myservo1.slowmove(0,127);  // set the servo to its initial position at fast speed
  myservo2.slowmove(0,127);  // set the servo to its initial position at fast speed
  delay(1000);
} 

void loop() {
  // slowmove(degrees 0-180, speed 1-255)
  /* On the RC servos tested, speed differences above 127 
     cannot be noticed due to the mechanical limitations of 
     the servo. However, feel free to experiment with the 
     speed value that best suits the response of your servo 
     based on its mechanical and electronic characteristics 
  */
  myservo1.slowmove(180,127);  // move the servo to 180, fast speed                                      
  myservo2.slowmove(180,127);  // move the servo to 180, fast speed
  delay(1000);  // sets the time the servo needs to reach the position and stay there

  myservo1.slowmove(0,30);  // move the servo to 0, slow speed
  myservo2.slowmove(0,30);  // move the servo to 0, slow speed
  delay(3700);  // sets the time the servo needs to reach the position and stay there
} 
