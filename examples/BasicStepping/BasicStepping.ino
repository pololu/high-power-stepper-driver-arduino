/* This example shows basic use of the DRV8711 stepper motor driver.

   It shows how to initialize the driver, set the current limit, set 
   the micro-stepping mode, and enable the driver. It shows how to send 
   pulses to the STEP pin to get the driver to take steps and how to 
   switch directions using the DIR pin.

   Before using this example be sure the change the setTorque and setGain 
   lines to have an appropriate current limit for your system.
   
 */

#include <SPI.h>
#include <DRV8711.h>

const uint8_t slaveSelectPin = 2;
const uint8_t stepPin = 3;
const uint8_t resetPin = 4;
const uint8_t dirPin = 5;
const uint8_t sleepPin = 6;

DRV8711 stepper;

void setup() 
{
  SPI.begin();
  stepper.init(slaveSelectPin);
  delay(1);
  
  // Drive the RST, STEP, and DIR pins low initially. Drive the nSLP pin high initially.
  digitalWrite(resetPin, LOW);
  pinMode(resetPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  pinMode(stepPin, OUTPUT);
  digitalWrite(dirPin, LOW);
  pinMode(dirPin, OUTPUT);
  digitalWrite(sleepPin, HIGH);
  pinMode(sleepPin, OUTPUT);

  // Reset the driver to it's default settings.
  digitalWrite(resetPin, HIGH);
  delay(1);
  digitalWrite(resetPin, LOW);
  delay(1);

  // Set the current limit. You should change the numbers here to
  // an appropriate value for your particular system.
  // The current limit is given by the equation below:
  // IFS = (2.75 * TORQUE)/(256 * ISGAIN * .03)
  stepper.setTorque(56);
  stepper.setGain(20);

  // Set the step mode and enable the driver.
  stepper.setStepMode(32);
  stepper.enableDriver();
}

void loop() 
{
  // Step in the default direction 1000 times.
  setDirection(0);
  for(unsigned int x = 0; x < 1000; x++)
  {
    step();
  }
  
  // Wait for 300 ms.
  delay(300);

  // Step in the other direction 1000 times.
  setDirection(1);
  for(unsigned int x = 0; x < 1000; x++)
  {
    step();
  }

  // Wait for 300 ms.
  delay(300);
}

// Sends a pulse on the STEP pin to tell the driver to take
// one step, and also delays to control the speed of the motor.
void step()
{
  // The NXT/STEP minimum high pulse width is 2 microseconds.
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(3);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(3);

  // The delay here controls the stepper motor's speed.  You can
  // increase the delay to make the stepper motor go slower.  If
  // you decrease the delay, the stepper motor will go fast, but
  // there is a limit to how fast it can go before it starts
  // missing steps.
  delayMicroseconds(2000);
}

void setDirection(bool dir)
{
  // The NXT/STEP pin must not change for at least 0.5
  // microseconds before and after changing the DIR pin.
  delayMicroseconds(1);
  digitalWrite(dirPin, dir);
  delayMicroseconds(1);
}
