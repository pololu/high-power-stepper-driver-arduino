# High-Power Stepper Motor Driver library for Arduino

Version: 1.0.0<br/>
Release date: 2019-06-04<br/>
[www.pololu.com](https://www.pololu.com/)

## Summary

This is a library for the Arduino IDE that helps interface with a [Pololu
High-Power Stepper Motor Driver 36v4][hpsd-36v4].  It uses the [Arduino
SPI][spi] library to communicate with the SPI interface (SCS, SCLK, SDATI, and
SDATO) of the driver.

## Supported platforms

This library is designed to work with the Arduino IDE versions 1.8.x or later;
we have not tested it with earlier versions.  This library should support any
Arduino-compatible board, including the [Pololu A-Star controllers][a-star].

## Getting started

### Hardware

The HighPowerStepperDriver library supports Pololu's [High-Power Stepper Motor
Driver 36v4][hpsd-36v4].  Before continuing, careful reading of its product
page is recommended.

You will need to connect your motor, motor power, and IOREF as described on the
product page.  You should also make the following connections between the
Arduino and the driver:

| Arduino       | High-Power Stepper Motor Driver |
|---------------|---------------------------------|
| Digital pin 2 | DIR                             |
| Digital pin 3 | STEP                            |
| Digital pin 4 | SCS                             |
| SCK           | SCLK                            |
| MOSI          | SDATI                           |
| MISO          | SDATO                           |
| GND           | GND                             |

The SDATO pin is only needed if you want to read information back from the
stepper driver.  Since the motor can be stepped and its direction changed using
the SPI interface, it is possible to use the driver without connecting the STEP
and DIR pins, and they are not used in every example.

The SPI pins (MOSI, MISO, and SCK) on Arduino-compatible boards are sometimes
not labeled.  You should refer to the documentation for your particular board
to find the locations of these pins.

### Software

You can use the Library Manager to install this library:

1. In the Arduino IDE, open the "Sketch" menu, select "Include Library", then
   "Manage Libraries...".
2. Search for "HighPowerStepperDriver".
3. Click the HighPowerStepperDriver entry in the list.
4. Click "Install".

If this does not work, you can manually install the library:

1. Download the [latest release archive from GitHub][github] and decompress it.
2. Rename the folder "high-power-stepper-driver-arduino-arduino-xxxx" to
   "HighPowerStepperDriver".
3. Drag the "HighPowerStepperDriver" folder into the "libraries" directory
   inside your Arduino sketchbook directory.  You can view your sketchbook
   location by opening the "File" menu and selecting "Preferences" in the
   Arduino IDE.  If there is not already a "libraries" folder in that location,
   you should make the folder yourself.
4. After installing the library, restart the Arduino IDE.

## Examples

Several example sketches are available that show how to use the library. You
can access them from the Arduino IDE by opening the "File" menu, selecting
"Examples", and then selecting "HighPowerStepperDriver". If you cannot find the
examples, the library was probably installed incorrectly and you should retry
the installation instructions above.

## Documentation

For complete documentation of this library, including many features that were
not mentioned here, see [the high-power-stepper-driver-arduino
documentation][doc].  If you are already on that page, see the
HighPowerStepperDriver class reference.

## Version history

* 1.0.0 (2019-06-04): Original release.

[a-star]: https://www.pololu.com/a-star
[arduino-uno]: https://www.pololu.com/product/2191
[arduino-leonardo]: https://www.pololu.com/product/2191
[doc]: https://pololu.github.io/high-power-stepper-driver-arduino/
[github]: https://github.com/pololu/high-power-stepper-driver-arduino/releases
[hpsd-36v4]: https://www.pololu.com/product/3730
[spi]: http://www.arduino.cc/en/Reference/SPI
