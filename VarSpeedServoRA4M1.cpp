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

#if defined(ARDUINO_ARCH_RENESAS)

#include "Arduino.h"
#include "VarSpeedServoRA4M1.h"
#include "math.h"
#include "FspTimer.h"

#define TRIM_DURATION       2  // compensation ticks to trim adjust for digitalWrite delays

#define SERVO_MAX_SERVOS            (_Nbr_16timers  * SERVOS_PER_TIMER)
#define SERVO_INVALID_INDEX         (255)
// Lower the timer ticks for finer resolution
#define SERVO_US_PER_CYCLE          (20000)
#define SERVO_IO_PORT_ADDR(pn)      &((R_PORT0 + ((uint32_t) (R_PORT1 - R_PORT0) * (pn)))->PCNTR3)
#define SERVO_MIN_CYCLE_OFF_US      50

// Sequence vars
servoSequencePoint initSeq[] = {{0,100},{45,100}};

// Keep track of the total number of servos attached
static size_t n_servos=0;
static ra_servo_t ra_servos[SERVO_MAX_SERVOS];

static FspTimer servo_timer;
static bool servo_timer_started = false;
void servo_timer_callback(timer_callback_args_t *args);

static uint32_t servo_ticks_per_cycle = 0;
static uint32_t min_servo_cycle_low = 0;
static uint32_t active_servos_mask = 0;
static uint32_t active_servos_mask_refresh = 0;

static uint32_t us_to_ticks(uint32_t time_us) {  // converts a time value from microseconds to ticks
  return ((float) servo_ticks_per_cycle / (float) SERVO_US_PER_CYCLE) * time_us;
}

static uint32_t ticks_to_us(uint32_t number_ticks) {  // converts a time value from ticks to microseconds 
  return ((float) SERVO_US_PER_CYCLE / (float) servo_ticks_per_cycle) * number_ticks;
}

static int servo_timer_config(uint32_t period_us)
{
  static bool configured = false;
  if (configured == false) {
    // Configure and enable the servo timer
    uint8_t type = 0;
    int8_t channel = FspTimer::get_available_timer(type);
    if (channel != -1) {
      servo_timer.begin(TIMER_MODE_PERIODIC, type, channel,
      1000000.0f/period_us, 50.0f, servo_timer_callback, nullptr);
      servo_timer.set_period_buffer(false);  // disable period buffering
      servo_timer.setup_overflow_irq(10);
      servo_timer.open();
      servo_timer.stop();
      // Read the timer's period count
      servo_ticks_per_cycle = servo_timer.get_period_raw();
      min_servo_cycle_low = us_to_ticks(SERVO_MIN_CYCLE_OFF_US);

      configured = true;
    }
  }
  return configured ? 0 : -1;
}

static int servo_timer_start()
{
  // Start the timer if it's not started
  if (servo_timer_started == false &&
    servo_timer.start() == false) {
    return -1;
  }
  servo_timer_started = true;
  return 0;
}

static int servo_timer_stop()
{
  // Start the timer if it's not started
  if (servo_timer_started == true &&
    servo_timer.stop() == false) {
    return -1;
  }
  servo_timer_started = false;
  return 0;
}

inline static void servo_timer_set_period(uint32_t period) {
  servo_timer.set_period(period);
}

void servo_timer_callback(timer_callback_args_t *args)
{
  (void)args; // remove warning
  static uint8_t channel = SERVO_MAX_SERVOS;
  static uint8_t channel_pin_set_high = 0xff;
  static uint32_t ticks_accum = 0;

  // See if we need to set a servo back low
  if (channel_pin_set_high != 0xff) {
    *ra_servos[channel_pin_set_high].io_port = ra_servos[channel_pin_set_high].io_mask << 16;
  }

  // Find the next servo to set high
  while (active_servos_mask_refresh) {

    // Extension for slowmove 
	  if (ra_servos[channel].speed) {
		  /* Increment ticks by speed until we reach the target. When the 
         target is reached, speed is set to 0 to disable that code
      */
		  if (ra_servos[channel].target > ra_servos[channel].period_ticks) {
			  ra_servos[channel].period_ticks += ra_servos[channel].speed;
			  if (ra_servos[channel].target <= ra_servos[channel].period_ticks) {
				  ra_servos[channel].period_ticks = ra_servos[channel].target;
				  ra_servos[channel].speed = 0;
			  }
		  }
		  else {
			  ra_servos[channel].period_ticks -= ra_servos[channel].speed;
			  if (ra_servos[channel].target >= ra_servos[channel].period_ticks) {
				  ra_servos[channel].period_ticks = ra_servos[channel].target;
			    ra_servos[channel].speed = 0;
			  }
		  }
	  }
	  // End of Extension for slowmove

    channel = __builtin_ctz(active_servos_mask_refresh);
    if (ra_servos[channel].period_us) {
      *ra_servos[channel].io_port = ra_servos[channel].io_mask;
      servo_timer_set_period(ra_servos[channel].period_ticks);
      channel_pin_set_high = channel;
      ticks_accum += ra_servos[channel].period_ticks;
      active_servos_mask_refresh &= ~(1 << channel);
      return;
    }
    active_servos_mask_refresh &= ~(1 << channel);
  }
  // Finished processing all servos, now delay to start of next pass
  ticks_accum += min_servo_cycle_low;
  uint32_t time_to_next_cycle;
  if (servo_ticks_per_cycle > ticks_accum) {
    time_to_next_cycle = servo_ticks_per_cycle - ticks_accum;
  } 
  else {
    time_to_next_cycle = min_servo_cycle_low;
  }
  ticks_accum = 0;
  servo_timer_set_period(time_to_next_cycle);
  channel_pin_set_high = 0xff;
  active_servos_mask_refresh = active_servos_mask;
}

VarSpeedServoRA4M1::VarSpeedServoRA4M1()
{
  servoIndex = SERVO_INVALID_INDEX;
  
  this->curSeqPosition = 0;  // sequence vars
  this->curSequence = initSeq;  // sequence vars
}

uint8_t VarSpeedServoRA4M1::attach(int pin)
{
  return attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

bool VarSpeedServoRA4M1::attached()
{
  return (servoIndex != SERVO_INVALID_INDEX);
}

uint8_t VarSpeedServoRA4M1::attach(int pin, int min, int max)
{
  //assert(pin < NUM_DIGITAL_PINS); ?
  if (n_servos == SERVO_MAX_SERVOS) {
    return 0;
  }

  // Configure the servo timer
  if (servo_timer_config(SERVO_US_PER_CYCLE) != 0) {
    return 0;
  }

  // Try to find a free servo slot
  ra_servo_t *servo = NULL;
  bsp_io_port_pin_t io_pin = g_pin_cfg[pin].pin;
  for (size_t i=0; i<SERVO_MAX_SERVOS; i++) {
    servo = &ra_servos[i];
    if (servo->period_us == 0) {
      n_servos++;
      servoIndex = i;
      servo->period_min = min;
      servo->period_max = max;
      servo->io_mask = (1U << (io_pin & 0xFF));
      servo->io_port = SERVO_IO_PORT_ADDR(((io_pin >> 8U) & 0xFF));
      active_servos_mask |= (1 << i);  // update mask of servos that are active
      writeMicroseconds(DEFAULT_PULSE_WIDTH);
      break;
    }
  }

  if (servoIndex == SERVO_INVALID_INDEX) {
    return 0;
  }

  // Configure GPIO pin for the servo
  R_IOPORT_PinCfg(&g_ioport_ctrl, io_pin,
  IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_HIGH);

  // Start the timer if it's not started
  if (servo_timer_start() != 0) {
    return 0;
  }
  return 1;
}

void VarSpeedServoRA4M1::detach()
{
  if (servoIndex != SERVO_INVALID_INDEX) {
    ra_servo_t *servo = &ra_servos[servoIndex];
    servo_timer_stop();
    servo->period_us = 0;
    active_servos_mask &= ~(1 << servoIndex);  // update mask of servos that are active
    servoIndex = SERVO_INVALID_INDEX;
    if (--n_servos) {
      servo_timer_start();
    }
  }
}

void VarSpeedServoRA4M1::write(int angle)
{
  byte channel = this->servoIndex; 
  ra_servos[channel].angle = angle; 
  
  if (servoIndex != SERVO_INVALID_INDEX) {
    ra_servo_t *servo = &ra_servos[servoIndex];
    angle = constrain(angle, 0, 180);
    writeMicroseconds(map(angle, 0, 180, servo->period_min, servo->period_max));
  }
}

int VarSpeedServoRA4M1::read()
{
  if (servoIndex != SERVO_INVALID_INDEX) {
    ra_servo_t *servo = &ra_servos[servoIndex];
    return map(readMicroseconds()+1, servo->period_min, servo->period_max, 0, 180);
  }
  return 0;
}

void VarSpeedServoRA4M1::writeMicroseconds(int us)
{
  byte channel = this->servoIndex;   
  if (servoIndex != SERVO_INVALID_INDEX) {
    ra_servo_t *servo = &ra_servos[servoIndex];
    servo->period_us = constrain(us, servo->period_min, servo->period_max);
    servo->period_ticks = us_to_ticks(servo->period_us);
    
    // Extension for slowmove
    // Disable slowmove logic
    ra_servos[channel].speed = 0; 
    // End of Extension for slowmove
  }
}

int VarSpeedServoRA4M1::readMicroseconds()
{
  unsigned int pulsewidth;
  ra_servo_t *servo = &ra_servos[servoIndex]; 
  if (servoIndex != SERVO_INVALID_INDEX) 
    pulsewidth = ticks_to_us(servo->period_ticks) + TRIM_DURATION;  
  else
    pulsewidth  = 0;

  return pulsewidth;
}

// Extension for slowmove
/*
  write(angle, speed) - Just like write but at reduced speed

  angle - Target position for the servo. Identical use as value of the function write
  speed - Speed at which to move the servo
          speed=0 - Full speed, identical to write
          speed=1 - Minimum speed
          speed=255 - Maximum speed
*/
void VarSpeedServoRA4M1::write(int angle, uint8_t speed) {
  /* This fuction is a copy of write and writeMicroseconds but value will be saved
     in target instead of in ticks in the servo structure and speed will be save
     there too
  */
  byte channel = this->servoIndex; 
  ra_servos[channel].angle = angle;

  if (speed) { 
    if (angle < MIN_PULSE_WIDTH) {
      ra_servo_t *servo = &ra_servos[servoIndex];
      // Treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
      // Updated to use constrain instead of if, pva
      angle = constrain(angle, 0, 180);
      angle = map(angle, 0, 180, servo->period_min, servo->period_max);
    }
    
    // Calculate and store the values for the given channel
    byte channel = this->servoIndex;
    if( (channel >= 0) && (channel < SERVO_MAX_SERVOS) ) {  // ensure channel is valid
      ra_servo_t *servo = &ra_servos[servoIndex];
      // Updated to use constrain instead of if, pva
      angle = constrain(angle, servo->period_min, servo->period_max);

      angle = angle - TRIM_DURATION;
      angle = us_to_ticks(angle);  // convert to ticks after compensating for interrupt overhead 
                                 
      // Set speed and direction
      ra_servos[channel].target = angle; 
      ra_servos[channel].speed = speed; 
    }
  } 
  else {
    write (angle);
  }
}

void VarSpeedServoRA4M1::write(int angle, uint8_t speed, bool wait) {
  write(angle,speed);
  
  if (wait) {  // block until the servo is at its new position
    if (angle < MIN_PULSE_WIDTH) {
      while (read() != angle) {
        delay(5);
      }
    } else {
      while (readMicroseconds() != angle) {
        delay(5);
      }
    }
  }
} 

void VarSpeedServoRA4M1::stop() {
  write(read());
}

void VarSpeedServoRA4M1::slowmove(int angle, uint8_t speed) {
  // Legacy function to support original version of VarSpeedServo
  write(angle, speed);
}

// End of Extension for slowmove

uint8_t VarSpeedServoRA4M1::sequencePlay(servoSequencePoint sequenceIn[], uint8_t numPositions, bool loop, uint8_t startPos) {
  uint8_t oldSeqPosition = this->curSeqPosition;

  if( this->curSequence != sequenceIn) {
    //Serial.println("newSeq");
    this->curSequence = sequenceIn;
    this->curSeqPosition = startPos;
    oldSeqPosition = 255;
  }

  if (read() == sequenceIn[this->curSeqPosition].position && this->curSeqPosition != CURRENT_SEQUENCE_STOP) {
    this->curSeqPosition++;

    if (this->curSeqPosition >= numPositions) {  // at the end of the loop
      if (loop) {  // reset to the beginning of the loop
        this->curSeqPosition = 0;
      } else {  // stop the loop
        this->curSeqPosition = CURRENT_SEQUENCE_STOP;
      }
    }
  }

  if (this->curSeqPosition != oldSeqPosition && this->curSeqPosition != CURRENT_SEQUENCE_STOP) {
    /* CURRENT_SEQUENCE_STOP position means the animation has ended, and should no longer be played
       otherwise move to the next position 
    */
    write(sequenceIn[this->curSeqPosition].position, sequenceIn[this->curSeqPosition].speed);
    //Serial.println(this->seqCurPosition);
  }

  return this->curSeqPosition;
}

uint8_t VarSpeedServoRA4M1::sequencePlay(servoSequencePoint sequenceIn[], uint8_t numPositions) {
  return sequencePlay(sequenceIn, numPositions, true, 0);
}

void VarSpeedServoRA4M1::sequenceStop() {
  write(read());
  this->curSeqPosition = CURRENT_SEQUENCE_STOP;
}

// To be used only with "write(angle, speed)"
void VarSpeedServoRA4M1::wait() {
  byte channel = this->servoIndex;
  int angle = ra_servos[channel].angle;

  // Wait until is done
  if (angle < MIN_PULSE_WIDTH) {
    while (read() != angle) {
      delay(5);
    }
  } else {
    while (readMicroseconds() != angle) {
      delay(5);  
    }
  }
}

bool VarSpeedServoRA4M1::isMoving() { 
  byte channel = this->servoIndex;
  int angle = ra_servos[channel].angle; 

  if (angle < MIN_PULSE_WIDTH) {
    if (read() != angle) {
      return true;
    }
  } else {
    if (readMicroseconds() != angle) {
      return true;
    }
  }
  return false;
}

#endif  // defined(ARDUINO_ARCH_RENESAS)
