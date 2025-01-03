[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/E1E515TIBE)

# VarSpeedServoRA4M1.h


Copyright (c) **2024 Kaled Souky**. All rights reserved.

This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.

----------------------------

**VarSpeedServoRA4M1** is a **library** for **Arduino** (developed by **Kaled Souky** in **2024**), based on the **Servo library** (developed by **Michael Margolis** in **2009**) and **VarSpeedServo library** (developed by **Korman** in **2010** and updated by **Philip van Allen** in **2013**). Specifically designed for the **Renesas RA4M1 microcontroller architecture (Arm Cortex-M4)**, present on the **Arduino UNO R4 Minima** and **Arduino UNO R4 WiFi** boards.

**VarSpeedServoRA4M1** is an **interrupt-driven** library using **16-bit timers**, allowing the use of up to **12 servos** moving asynchronously or synchronously. In addition to setting the position, you can set the speed of a movement, optionally wait until the servo movement is completed, and create sequences of movements that execute asynchronously or synchronously.

The **32-bit Renesas RA4M1 processor** operating at **48 MHz** elevates servo control to a new level, providing **higher PWM resolution** compared to the **ATmega328P** used in **Arduino UNO R3**. Thanks to this power, **VarSpeedServoRA4M1** fully exploits the microcontroller's capabilities. With its **simple and intuitive syntax**, this **library** facilitates precise control over servo position and speed, even for beginners. Creating complex movement sequences is straightforward with just a few commands.


> [!WARNING]
> **"External devices with a high current draw (e.g. servo motors) should never be powered via the 5 V pin. It is mainly intended for devices drawing lower current such as sensor modules"**.

> For more information about the **Arduino UNO R4 Minima** and **Arduino UNO R4 WiFi** boards, please check the following links from the **official Arduino site**:
* [Arduino UNO R4 Minima Cheat Sheet](https://docs.arduino.cc/tutorials/uno-r4-minima/cheat-sheet/)  
* [Arduino UNO R4 WiFi Cheat Sheet](https://docs.arduino.cc/tutorials/uno-r4-wifi/cheat-sheet/)


> [!TIP] 
> How to connect power and control servo motors with your Arduino board. Please check the link of **Arduino official site**: 
 * [Servo Motor Basics with Arduino](https://docs.arduino.cc/learn/electronics/servo-motors/)


## Features:

* Supports up to 12 servos.
* Allows synchronous and asynchronous movement of all servos.
* Allows you to configure the **"position"** and **"speed"** of each servo individually.
* Includes a **"wait"** parameter to wait for the servo to complete its movement. **Philip van Allen**, in his update, incorporated it as a boolean variable within the **write()** function, and also as the independent `wait()` function. This parameter is not included as a variable in **Korman's** `slowmove()` function.
* The **write()** function now includes the **"speed"** and **"wait"** parameters, which can optionally be used in conjunction 
 with the **"position"** parameter to control servo movement.
* The `slowmove()` function is retained for compatibility with the **Korman** version. It only handles the **"position"** and **"speed"** parameters. In this case, the waiting time must be configured (using, for example, the function `delay()`, `delayMicroseconds()`, `micros()` or `millis()`), so that the servo completes the position and maintains it for the desired time. The `slowmove()` function does not incorporate the **"wait"** variable introduced by **Philip van Allen** in the `write()` function.
* Allows the creation of **position sequences**.


## Class methods

A servo is activated by creating an instance of the **VarSpeedServoRA4M1 class** passing the desired pin to the `attach()` method. The servos are pulsed in the background using the value most recently written using the `write()` method

**Note** that **analogWrite of PWM** on pins associated with the timer are disabled when the first servo is attached.
Timers are seized as needed in groups of 12 servos - 24 servos use two timers, 48 servos will use four.
The sequence used to seize timers is defined in the content of the **"ServoTimers.h"** file for **Renesas** of the **Arduino Servo library**.

  
**VarSpeedServoRA4M1** - **Class** for manipulating servo motors connected to **Arduino** pins. **Methods**:
  ```
  VarSpeedServoRA4M1() - Creates a new Servo object and prepares it for controlling a servo motor
  attach(pin ) - Attaches a servo motor to an i/o pin
  attach(pin, min, max  ) - Attaches to a pin setting min and max values in microseconds default min is 544, max is 2400  

  write(value) - Sets the servo angle in degrees (invalid angle that is valid as pulse in microseconds is treated as microseconds)
  write(value, speed) - speed varies the speed of the move to new position 0=full speed, 1-255 slower to faster
  write(value, speed, wait) - wait is a boolean that, if true, causes the function call to block until move is complete

  writeMicroseconds() - Sets the servo pulse width in microseconds 
  read() - Gets the last written servo pulse width as an angle between 0 and 180 
  readMicroseconds() - Gets the last written servo pulse width in microseconds (was read_us() in first release)
  attached() - Returns true if there is a servo attached 
  detach() - Stops an attached servos from pulsing its i/o pin 

  slowmove(value, speed) - The same as write(value, speed), retained for compatibility with Korman's version

  stop() - stops the servo at the current position

  sequencePlay(sequence, sequencePositions) - play a looping sequence starting at position 0
  sequencePlay(sequence, sequencePositions, loop, startPosition) - play sequence with number of positions, loop if true, start at position
  sequenceStop() - stop sequence at current position
  wait() - wait for movement to finish
  isMoving() - return true if servo is still moving
  ```
  
  

## Examples of use:

**Sample code 1:** Sweeps a servo motor from 0 to 180 degrees at fast speed and back from 180 to 0 degrees at slow speed, in an infinite loop.

The `write()` function now includes the **"speed"** and **"wait"** parameters, which can optionally be used in conjunction with the **"position"** parameter to control servo movement. 

The **"wait"** parameter is used to wait for the servo to complete its movement. It is also available as a standalone `wait()` function.

```

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

```

**Sample code 2:** Sweeps a servo motor from 0 to 180 degrees at fast speed and back from 180 to 0 degrees at slow speed, in an infinite loop.

The `slowmove()` function only handles the **"position"** and **"speed"** parameters. In this case, the wait time must be set (using, for example, the `delay()`, `millis()`, `micros()`, or `delayMicroseconds()` functions) to ensure the servo completes the position and maintains it for the desired time. The `slowmove()` function does not incorporate the **"wait"** variable.

This example demonstrates the use of the `delay()` function as a timeout with `slowmove()`. The exact delay value may require some experimentation (usually in the millisecond range) to ensure that the servo has enough time to reach its target position and hold it for as long as necessary, until receiving the next instruction.

```

#include <VarSpeedServoRA4M1.h> 
 
VarSpeedServoRA4M1 myservo;  // create servo object to control a servo 
// A maximum of twelve servo objects can be created 
 
const int servoPin = 9;  // the digital pin used for the servo
 
void setup() { 
  myservo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  myservo.slowmove(0,127);  // set the servo to its initial position at fast speed
  delay(1000);  // sets the time the servo needs to reach the position and stay there
} 

void loop() {
  // slowmove(degrees 0-180, speed 1-255)
  /* On the RC servos tested, speed differences above 127 
     cannot be noticed due to the mechanical limitations of 
     the servo. However, feel free to experiment with the 
     speed value that best suits the response of your servo 
     based on its mechanical and electronic characteristics 
  */
  myservo.slowmove(180,127);  // move the servo to 180, fast speed
  delay(1000);  // sets the time the servo needs to reach the position and stay there
  myservo.slowmove(0,30);  // move the servo to 0, slow speed
  delay(3700);  // sets the time the servo needs to reach the position and stay there
} 

```

Additional examples are included in the distribution and are available in the **Arduino IDE Examples** section.



> [!NOTE]
> The **VarSpeedServoRA4M1 library** is not included in the standard **Arduino** distribution. You must download and install it separately.

> There are two methods to achieve this:
* Import a ZIP library
* Manual installation

> These methods are described on the **official Arduino site**:

* [Installing Libraries](https://www.arduino.cc/en/Guide/libraries) 

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/E1E515TIBE)
