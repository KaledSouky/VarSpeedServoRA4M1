/*
  VarSpeedServoRA4M1.cpp - Version 2
  Copyright (c) 2024 Kaled Souky. All rights reserved.

  This library is free software; you can redistribute it 
  and/or modify it under the terms of the GNU Lesser 
  General Public License as published by the Free Software 
  Foundation; either version 2.1 of the License, or 
  (at your option) any later version.

  This library is distributed in the hope that it will be 
  useful, but WITHOUT ANY WARRANTY; without even the 
  implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE. See the GNU Lesser General Public 
  License for more details.

  You should have received a copy of the GNU Lesser 
  General Public License along with this library; if not, 
  write to the Free Software Foundation, Inc., 51 Franklin 
  St, Fifth Floor, Boston, MA 02110-1301 USA.

  -------------------------------------------------------

  VarSpeedServoRA4M1 is a library for Arduino, based on the
  Servo library (developed by Michael Margolis in 2009) and 
  VarSpeedServo library (developed by Korman in 2010 and 
  updated by Philip van Allen in 2013). Specifically 
  designed for the Renesas RA4M1 microcontroller 
  architecture (Arm Cortex-M4), present on the Arduino UNO 
  R4 Minima and Arduino UNO R4 WiFi boards.

  VarSpeedServoRA4M1 is an interrupt-driven library using 
  16-bit timers, allowing the use of up to 12 servos moving
  asynchronously or synchronously. In addition to setting 
  the position, you can set the speed of a movement, 
  optionally wait until the servo movement is completed, and 
  create sequences of movements that execute asynchronously 
  or synchronously.

  The 32-bit Renesas RA4M1 processor operating at 48 MHz 
  elevates servo control to a new level, providing higher 
  PWM resolution compared to the ATmega328P used in Arduino 
  UNO R3. Thanks to this power, VarSpeedServoRA4M1 fully 
  exploits the microcontroller's capabilities. With its 
  simple and intuitive syntax, this library facilitates 
  precise control over servo position and speed, even for 
  beginners. Creating complex movement sequences is 
  straightforward with just a few commands.
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

  -------------------------------------------------------

  Features:
  
  * Supports up to 12 servos.
  * Allows synchronous and asynchronous movement of all 
    servos.
  * Allows you to configure the "position" and "speed" of 
    each servo individually.
  * Includes a "wait" parameter to wait for the servo to 
    complete its movement. Philip van Allen, in his update, 
    incorporated it as a boolean variable within the write() 
    function, and also as the independent wait() function. 
    This parameter is not included as a variable in Korman's 
    slowmove() function.
  * The write() function now includes the "speed" and 
    "wait" parameters, which can optionally be used in 
    conjunction with the "position" parameter to control 
    servo movement.
  * The slowmove() function is retained for compatibility 
    with the Korman version. It only handles the "position" 
    and "speed" parameters. In this case, the waiting time
    must be configured (using, for example, the function 
    delay(), delayMicroseconds(), micros() or millis()), so
    that the servo completes the position and maintains it 
    for the desired time. The slowmove() function does not 
    incorporate the "wait" variable introduced by Philip 
    van Allen in the write() function.
  * Allows the creation of position sequences.

  -------------------------------------------------------

  Class methods:

  A servo is activated by creating an instance of the 
  VarSpeedServoRA4M1 class passing the desired pin to the 
  attach() method. The servos are pulsed in the background 
  using the value most recently written using the write() 
  method.

  Note that analogWrite of PWM on pins associated with the 
  timer are disabled when the first servo is attached.
  Timers are seized as needed in groups of 12 servos - 24 
  servos use two timers, 48 servos will use four.
  The sequence used to seize timers is defined in the 
  content of the "ServoTimers.h" file for Renesas of the 
  Arduino Servo library.

  VarSpeedServoRA4M1 - Class for manipulating servo motors 
  connected to Arduino pins. Methods:

  VarSpeedServoRA4M1() - Creates a new Servo object and 
  prepares it for controlling a servo motor
 
  attach(pin ) - Attaches a servo motor to an i/o pin
  
  attach(pin, min, max  ) - Attaches to a pin setting min 
  and max values in microseconds default min is 544, max 
  is 2400  

  write(angle) - Sets the servo angle in degrees (invalid 
  angle that is valid as pulse in microseconds is treated 
  as microseconds)
  
  write(angle, speed) - speed varies the speed of the move 
  to new position 0=full speed, 1-255 slower to faster
  
  write(angle, speed, wait) - wait is a boolean that, if 
  true, causes the function call to block until move is 
  complete

  writeMicroseconds() - Sets the servo pulse width in 
  microseconds 
  
  read() - Gets the last written servo pulse width as an 
  angle between 0 and 180 
  
  readMicroseconds() - Gets the last written servo pulse 
  width in microseconds (was read_us() in first release)
  
  attached() - Returns true if there is a servo attached 
  
  detach() - Stops an attached servos from pulsing its i/o 
  pin. 

  slowmove(angle, speed) - The same as write(angle, speed), 
  retained for compatibility with Korman's version

  stop() - Stops the servo at the current position

  sequencePlay(sequence, sequencePositions) - Play a looping
  sequence starting at position 0
  
  sequencePlay(sequence, sequencePositions, loop, 
  startPosition) - Play sequence with number of positions, 
  loop if true, start at position
  
  sequenceStop() - Stop sequence at current position
  
  wait() - Wait for movement to finish
  
  isMoving() - Return true if servo is still moving
*/

#ifndef VarSpeedServoRA4M1_h
#define VarSpeedServoRA4M1_h

#include <inttypes.h>

/* 
 * Defines for 16 bit timers used with VarSpeedServoRA4M1 library 
 *
 * If _useTimerX is defined then TimerX is a 16 bit timer on the current board
 * timer16_Sequence_t enumerates the sequence that the timers should be allocated
 * _Nbr_16timers indicates how many 16 bit timers are available
*/

// Architecture specific include
#if defined(ARDUINO_ARCH_RENESAS)
/* Copied from the "ServoTimers.h" file for Renesas of the Arduino Servo library
   for Renesas RA4M1 (Arm Cortex-M4):
*/
#define _Nbr_16timers 1

#else
#error "This library only supports boards with a Renesas RA4M1 processor."
#endif

#define VarSpeedServoRA4M1_VERSION  2 // software version of this library

#define MIN_PULSE_WIDTH           544 // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH          2400 // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH      1500 // default pulse width when servo is attached
#define SERVOS_PER_TIMER           12 // the maximum number of servos controlled by one timer 
#define CURRENT_SEQUENCE_STOP     255 // used to indicate the current sequence is not used and sequence should stop

// Internal Servo sturct to keep track of RA configuration
typedef struct {
  uint32_t period_us;  // servo period in microseconds
  uint32_t period_min;  // store min pulse width here, because min in Servo class are not wide enough for the pulse width
  uint32_t period_max;  // store max pulse width here, because max in Servo class are not wide enough for the pulse width   
  uint32_t period_ticks;  // period period_count in timer ticks
  volatile uint32_t *io_port;  // Pointer to internal FSP GPIO port control register
  uint32_t io_mask;  // Bit mask to select control pins of FSP GPIO port
  volatile unsigned int ticks;  // represents the time in ticks to synchronize the servo pulses
  unsigned int angle;  // for external wait extension 
  unsigned int target;  // for slowmove extension
  uint8_t speed;  // for slowmove extension
} ra_servo_t;

typedef struct {
  uint8_t position;  // for sequences
  uint8_t speed;  // for sequences
} servoSequencePoint;

class VarSpeedServoRA4M1
{
public:
  VarSpeedServoRA4M1();
  uint8_t attach(int pin);  // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  uint8_t attach(int pin, int min, int max);  // as above but also sets min and max values for writes 
  void detach();  // stops an attached servos from pulsing its i/o pin 
  
  void write(int angle);  // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds 
  void write(int angle, uint8_t speed);  // move to given position at reduced speed
  /* speed=0 is identical to write, speed=1 slowest and speed=255 fastest. On the 
     RC servos tested, speed differences above 127 cannot be noticed due to the 
     mechanical limitations of the servo
  */
  void write(int angle, uint8_t speed, bool wait);  // wait parameter causes call to block until move completes
  void writeMicroseconds(int us);  // write pulse width in microseconds 
  void slowmove(int angle, uint8_t speed);  // support for legacy slowmove function
  void stop();  // stop the servo where it is

  int read();  // returns current pulse width as an angle between 0 and 180 degrees
  int readMicroseconds();  // returns current pulse width in microseconds for this servo (was read_us() in first release)
  bool attached();  // return true if this servo is attached, otherwise false 
  
  uint8_t sequencePlay(servoSequencePoint sequenceIn[], uint8_t numPositions, bool loop, uint8_t startPos);
  uint8_t sequencePlay(servoSequencePoint sequenceIn[], uint8_t numPositions);  // play a looping sequence starting at position 0
  void sequenceStop();  // stop movement
  void wait();  // wait for movement to finish
  bool isMoving();  // return true if servo is still moving
private:
  uint8_t servoIndex;  // index into the channel data for this servo
  int8_t min;  // minimum is this value times 4 added to MIN_PULSE_WIDTH
  int8_t max;  // maximum is this value times 4 added to MAX_PULSE_WIDTH
  servoSequencePoint * curSequence;  // for sequences
  uint8_t curSeqPosition;  // for sequences
};

#endif

