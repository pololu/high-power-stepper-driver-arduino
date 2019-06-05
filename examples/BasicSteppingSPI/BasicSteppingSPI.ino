// This example shows basic use of a Pololu High Power Stepper Motor Driver.
//
// It shows how to initialize the driver, configure various settings, and enable
// the driver.  It shows how to step the motor and switch directions using the
// driver's SPI interface through the library's step() and setDirection() member
// functions.
//
// Since SPI is used to trigger steps and set the direction, connecting the
// driver's STEP and DIR pins is optional for this example.  However, note that
// using SPI control adds some overhead compared to using the STEP and DIR pins.
// In addition, since the library caches SPI register values, SPI control is
// more likely to re-enable the driver with the wrong settings (e.g. current
// limit) after a power interruption, although using the verifySettings() and
// applySettings() functions appropriately can help prevent this.
//
// Before using this example, be sure to change the setCurrentMilliamps36v4 line
// to have an appropriate current limit for your system.  Also, see this
// library's documentation for information about how to connect the driver:
//   http://pololu.github.io/high-power-stepper-driver

#include <SPI.h>
#include <HighPowerStepperDriver.h>

const uint8_t CSPin = 4;

// This period is the length of the delay between steps, which controls the
// stepper motor's speed.  You can increase the delay to make the stepper motor
// go slower.  If you decrease the delay, the stepper motor will go faster, but
// there is a limit to how fast it can go before it starts missing steps.
const uint16_t StepPeriodUs = 2000;

HighPowerStepperDriver sd;

void setup()
{
  SPI.begin();
  sd.setChipSelectPin(CSPin);

  // Give the driver some time to power up.
  delay(1);

  // Reset the driver to its default settings and clear latched status
  // conditions.
  sd.resetSettings();
  sd.clearStatus();

  // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
  // for most applications, and we find that it usually works well.
  sd.setDecayMode(HPSDDecayMode::AutoMixed);

  // Set the current limit. You should change the number here to an appropriate
  // value for your particular system.
  sd.setCurrentMilliamps36v4(1000);

  // Set the number of microsteps that correspond to one full step.
  sd.setStepMode(HPSDStepMode::MicroStep32);

  // Enable the motor outputs.
  sd.enableDriver();
}

void loop()
{
  // Step in the default direction 1000 times.
  sd.setDirection(0);
  for(unsigned int x = 0; x < 1000; x++)
  {
    sd.step();
    delayMicroseconds(StepPeriodUs);
  }

  // Wait for 300 ms.
  delay(300);

  // Step in the other direction 1000 times.
  sd.setDirection(1);
  for(unsigned int x = 0; x < 1000; x++)
  {
    sd.step();
    delayMicroseconds(StepPeriodUs);
  }

  // Wait for 300 ms.
  delay(300);
}