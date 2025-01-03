/*
  VarSpeedServoRA4M1Test
  By Philip van Allen <philvanallen.com> for the 
  VarSpeedServo.h library (October 2013)
  Adapted by Kaled Souky <kaledsouky2@gmail.com> for the 
  VarSpeedServoRA4M1.h library (December 2024)
  This example code is in the public domain.
  
  -------------------------------------------------------

  This code controls two servos asynchronously in a 
  sequence that repeats four times using a for loop. To 
  create an infinite loop, comment out or remove the 
  section where the detach() function is called.
  
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
const int servoPin2 = 8;  // the digital pin used for the second servo
 
 
void setup() {

 // Initialize serial:
 // Serial.begin(9600);
  
  myservo1.attach(servoPin1);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(servoPin2);  // attaches the servo on pin 8 to the servo object
} 
 
void loop() {
  
 // Created integer variables   
  int LEF = 0;
  int RIG = 180;
  /* On the RC servos tested, speed differences above 127 
     cannot be noticed due to the mechanical limitations of 
     the servo. However, feel free to experiment with the 
     speed value that best suits the response of your servo 
     based on its mechanical and electronic characteristics 
  */
  int SPEED1 = 127;
  int SPEED2 = 100;
  
 // The for loop determines the number of times a code block is repeated                                 
  for(int i = 0; i < 4; i++) {  // the condition i < 4 sets the number of repetitions to 4, this number can be changed
   
    myservo1.write(LEF, SPEED1);  // moves the servo to 0 at a speed of 127   
    myservo2.write(LEF, SPEED2);  // moves the servo to 0 at a speed of 100  
    myservo1.wait();  // wait until myservo1 have finished their movements before proceeding
    myservo2.wait();  // wait until myservo2 have finished their movements before proceeding
      
    myservo1.write(RIG, SPEED1);  // moves the servo to 180 at a speed of 127
    myservo1.wait();  // wait until myservo1 have finished their movements before proceeding
    
    myservo1.write(LEF, SPEED1);  // moves the servo to 0 at a speed of 127
    myservo2.write(RIG, SPEED2);  // moves the servo to 180 at a speed of 100
    myservo1.wait();  // wait until myservo1 have finished their movements before proceeding
    myservo2.wait();  // wait until myservo2 have finished their movements before proceedin
          
    myservo1.write(RIG, SPEED1);  // moves the servo to 180 at a speed of 127   
    myservo1.wait();  // wait until myservo1 have finished their movements before proceedin
    
  }
  
  ///* Disable this part of the code, if you want the sequence to repeat in an infinite loop
  delay(3000);  // 3 millisecond delay before using the detach function with servo objects
  myservo1.detach();  // disconnects the myservo1 object from the pin 9
  myservo2.detach();  // disconnects the myservo2 object from the pin 8
  //*/
}

