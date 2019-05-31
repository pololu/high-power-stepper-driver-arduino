/* This example shows basic use of the DRV8711 sd motor driver.

   It shows how to initialize the driver, set the current limit, set 
   the micro-stepping mode, and enable the driver. It shows how to send 
   pulses to the STEP pin to get the driver to take steps and how to 
   switch directions using the DIR pin.

   Before using this example be sure the change the setTorque and setGain 
   lines to have an appropriate current limit for your system.
   
 */

#include <SPI.h>
#include <HighPowerStepperDriver.h>

const uint8_t chipSelectPin = 2;

HighPowerStepperDriver sd;

void setup() 
{
  SPI.begin();
  sd.setChipSelectPin(chipSelectPin);
  delay(1);
  
  // Reset the driver to it's default settings.
  sd.resetSettings();
  
  sd.setDecayMode(HPSDDecayMode::AutoMixed);
  
  // Set the current limit. You should change the numbers here to
  // an appropriate value for your particular system.
  // The current limit is given by the equation below:
  // IFS = (2.75 * TORQUE)/(256 * ISGAIN * .03)
  sd.setCurrentLimit36v40(1000);

  // Set the step mode and enable the driver.
  sd.setStepMode(HPSDStepMode::MicroStep32);
  sd.enableDriver();
}

void loop() 
{
  // Step in the default direction 1000 times.
  sd.setDirection(0);
  for(unsigned int x = 0; x < 1000; x++)
  {
    sd.step();
  }
  
  // Wait for 300 ms.
  delay(300);

  // Step in the other direction 1000 times.
  sd.setDirection(1);
  for(unsigned int x = 0; x < 1000; x++)
  {
    sd.step();
  }

  // Wait for 300 ms.
  delay(300);
}
