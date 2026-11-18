# Overview

Tunamelt* is a minimalist meltybrain combat robot firmware. Unlike existing loop-based implementations, it uses an asynchronous architecture based on arkhipenko's TaskScheduler library (https://github.com/arkhipenko/TaskScheduler) to handle precise timing across two cores of an RP2040 microcontroller while remaining configurable and extensible. In addition, it uses sinusoidal trigonometric functions to provide smooth, proportional translation.

An example video can be seen here: https://www.youtube.com/watch?v=Yd3VEgTqDbg

*"Every day for the last three weeks you've been coming in here and you've been asking me how the tuna is. Now, it was crappy yesterday, it was crappy the day before and guess what? It hasn't changed." - Mia Toretto, The Fast and the Furious

# Requirements

## Hardware

The proof of concept robot "The Room-ba" uses the following hardware:

Seeed Xiao RP2040
H3LIS331DL accelerometer
FlySky FS-A8S receiver
2x Emax Bullet 12A ESCs (flashed with Bluejay firwmare - this is not required but I found it to perform better)
2x generic silver can 1104 motors
2s 300mAh lipo

A custom PCB is not required but is highly recommended for actual combat due to reliability issues with point to point soldering.

## Software

This software was written using the Arduino-Pico RP2040 core: https://github.com/earlephilhower/arduino-pico

following libraries (and their dependencies) were used:

https://github.com/arkhipenko/TaskScheduler
https://github.com/adafruit/Adafruit_LIS331
https://github.com/bastian2001/pico-bidir-dshot
https://github.com/adafruit/Adafruit_NeoPixel

# Configuration

The file "melty.h" contains various configuration options. These are as follows:

* SPEED_LO: Low speed mode duty cycle (/255). Lower spin speeds offer higher translation speeds at the expense of kinetic energy.
* SPEED_HI: High speed mode duty cycle.
* ACCEL_DIST: Accelerometer distance from centre of rotation in millimetres.
* HEADING_ANGLE: Number of degrees to display the heading LED for.
* LED_BRIGHTNESS: LED duty cycle (/255). Turn down to avoid burning out heading LEDs when run without a resistor.
* VBATT_R1: High side divider resistance in ohms
* VBATT_R2: Low side resistance in ohms
* LOW_BATT_MVOLTS: Low battery value in millivolts. Usually 3V/cell for lipo batteries.

Additionally, pin assignments can be set here.

# Theory of Operation

* The serial buffer is polled for a new radio packet every task scheduler loop. If one is received, it is parsed into control inputs. If one is not received for over 100ms the code enters failsafe state and the motor outputs are disabled.
* The accelerometer is read constantly at 100hz, ensuring the latest accelerometer data is always available. DShot packets are always sent to the ESCs, even if the values sent are zero, to ensure they do not enter failsafe/disconnect mode.
* When a valid radio packet is parsed, the main logic task is triggered. This function handles switching between tank drive and forward/reverse meltybrain modes, as well as accelerometer trim and battery status.
* In meltybrain mode the RPM calculated from the accelerometer readings is used to time the interval between each iteration of the meltybrain task. The interval calculatd in the main meltybrain task is used to schedule the heading LED and translation tasks, as well as the next iteration of the main meltybrain task. The steering input from the radio adds or subtracts a small amount of time to this interval, which gives the appearance of the heading LED turning as the steering wheel/stick is moved.
* Motor duty cycles are calculated 32 times per rotation of the robot. A sinusoidal function mapped across one rotation of the robot is used to calculate the motor duty cycles, with both wheels spinning at the baseline spin speed of the robot when perpendicular to the direction of travel, and at their maximum/minimum (the magnitude of the max/mins is based on the throttle input) when parallel to the direction of travel.

# RGB LED

As well as cool rainbows to make the robot go faster, the RGB LED is used to indicate status as follows:

* Rainbow: OK
* Blue: Receiver failsafe
* Red: Battery low voltage

# Minor Bugs/Future Improvements

* The heading LED shifts a few degrees during translation, with the amount of shift dependent on the throttle input. The fix could be as simple as adding a small adjustment factor to re-align it, but I don't find the shift large enough to be an issue.
* The heading LED doesn't track while the bot is spinning up. This is either due to the calculated RPM lagging the actual robot RPM, or the slew rate being limited by the low pass filter enabled in the accelerometer. Again I don't find this to personally be an issue, as a 150g bot spins up within a second or two, but this may be a bigger issue on larger weight class robots.
* My PCB features two accelerometers but I have only implemented one so far. The Project Liftoff team have written a very good document that covers their approch to an advanced dual-accelerometer solution: https://docs.google.com/document/d/1yK2wLSst71c1KfBYTjT9I6IOZBh7KStF1HXj2nAxNgw/
* Strafing movements are theoretically possible, but as I use a pistol grip transmitter I lack the number of hands required to operate my transmitter this way, thus I haven't implemented it.

# Other Information

Default radio channels:

1: Steering

2: Throttle (forwards/backwards)

3: Speed (3 positions, switch or potentiometer)

4: N/A

5: Accelerometer trim (Trim switch and/or potentiometer)

6: Spin direction (2 position switch)

While I use a programmable EdgeTX transmitter, these should be compatible with common non-programmable transmitters like the FlySky FS-i6.
