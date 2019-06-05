// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/

/// \file HighPowerStepperDriver.h
///
/// This is the main header file for the HighPowerStepperDriver library,
/// a library for controlling Pololu's High-Power Stepper Motor Drivers that are
/// based on the DRV8711.
///
/// For more information about this library, see:
///
///   https://github.com/pololu/high-power-stepper-driver-arduino
///
/// That is the main repository for this library.

#pragma once

#include <Arduino.h>
#include <SPI.h>


/// Addresses of control and status registers.
enum class HPSDRegAddr : uint8_t
{
  CTRL   = 0x00,
  TORQUE = 0x01,
  OFF    = 0x02,
  BLANK  = 0x03,
  DECAY  = 0x04,
  STALL  = 0x05,
  DRIVE  = 0x06,
  STATUS = 0x07,
};


/// This class provides low-level functions for reading and writing from the SPI
/// interface of a DRV8711 stepper motor controller IC.
///
/// Most users should use the HighPowerStepperDriver class, which provides a
/// higher-level interface, instead of this class.
class DRV8711SPI
{
public:
  /// Configures this object to use the specified pin as a chip select pin.
  ///
  /// You must use a chip select pin; the DRV8711 requires it.
  void setChipSelectPin(uint8_t pin)
  {
    csPin = pin;
    digitalWrite(csPin, LOW);
    pinMode(csPin, OUTPUT);
  }

  /// Reads the register at the given address and returns its raw value.
  uint16_t readReg(uint8_t address)
  {
    // Read/write bit and register address are the first 4 bits of the first
    // byte; data is in the remaining 4 bits of the first byte combined with
    // the second byte (12 bits total).

    selectChip();
    uint16_t dataOut = transfer((0x8 | (address & 0b111)) << 12);
    deselectChip();
    return dataOut & 0xFFF;
  }

  /// Reads the register at the given address and returns its raw value.
  uint16_t readReg(HPSDRegAddr address)
  {
    return readReg((uint8_t)address);
  }

  /// Writes the specified value to a register.
  void writeReg(uint8_t address, uint16_t value)
  {
    // Read/write bit and register address are the first 4 bits of the first
    // byte; data is in the remaining 4 bits of the first byte combined with
    // the second byte (12 bits total).

    selectChip();
    transfer(((address & 0b111) << 12) | (value & 0xFFF));

    // The CS line must go low after writing for the value to actually take
    // effect.
    deselectChip();
  }

  /// Writes the specified value to a register.
  void writeReg(HPSDRegAddr address, uint16_t value)
  {
    writeReg((uint8_t)address, value);
  }

private:

  SPISettings settings = SPISettings(500000, MSBFIRST, SPI_MODE0);

  uint16_t transfer(uint16_t value)
  {
    return SPI.transfer16(value);
  }

  void selectChip()
  {
    digitalWrite(csPin, HIGH);
    SPI.beginTransaction(settings);
  }

  void deselectChip()
  {
   SPI.endTransaction();
   digitalWrite(csPin, LOW);
  }

  uint8_t csPin;
};


/// Possible arguments to setStepMode().
enum class HPSDStepMode : uint16_t
{
  MicroStep256 = 256,
  MicroStep128 = 128,
  MicroStep64  =  64,
  MicroStep32  =  32,
  MicroStep16  =  16,
  MicroStep8   =   8,
  MicroStep4   =   4,
  MicroStep2   =   2,
  MicroStep1   =   1,
};

/// Possible arguments to setDecayMode().
enum class HPSDDecayMode : uint8_t
{
  Slow                = 0b000,
  SlowIncMixedDec     = 0b001,
  Fast                = 0b010,
  Mixed               = 0b011,
  SlowIncAutoMixedDec = 0b100,
  AutoMixed           = 0b101,
};

/// Bits that are set in the return value of readStatus() to indicate status
/// conditions.
///
/// See the DRV8711 datasheet for detailed descriptions of these status
/// conditions.
enum class HPSDStatusBit : uint8_t
{
  /// Overtemperature shutdown
  OTS = 0,

  /// Channel A overcurrent shutdown
  AOCP = 1,

  /// Channel B overcurrent shutdown
  BOCP = 2,

  /// Channel A predriver fault
  APDF = 3,

  /// Channel B predriver fault
  BPDF = 4,

  /// Undervoltage lockout
  UVLO = 5,

  /// Stall detected
  STD = 6,

  /// Latched stall detect
  STDLAT = 7,
};


/// This class provides high-level functions for controlling a DRV8711-based
/// High-Power Stepper Motor Driver.
class HighPowerStepperDriver
{
public:
  /// The default constructor.
  HighPowerStepperDriver()
  {
    // All settings set to power-on defaults
    ctrl   = 0xC10;
    torque = 0x1FF;
    off    = 0x030;
    blank  = 0x080;
    decay  = 0x110;
    stall  = 0x040;
    drive  = 0xA59;
  }

  /// Configures this object to use the specified pin as a chip select pin.
  /// You must use a chip select pin; the DRV8711 requires it.
  void setChipSelectPin(uint8_t pin)
  {
    driver.setChipSelectPin(pin);
  }

  /// Changes all of the driver's settings back to their default values.
  ///
  /// It is good to call this near the beginning of your program to ensure that
  /// there are no settings left over from an earlier time that might affect the
  /// operation of the driver.
  void resetSettings()
  {
    ctrl   = 0xC10;
    torque = 0x1FF;
    off    = 0x030;
    blank  = 0x080;
    decay  = 0x110;
    stall  = 0x040;
    drive  = 0xA59;
    applySettings();
  }

  /// Reads back the SPI configuration registers from the device and verifies
  /// that they are equal to the cached copies stored in this class.
  ///
  /// This can be used to verify that the driver is powered on and has not lost
  /// them due to a power failure.  The STATUS register is not verified because
  /// it does not contain any driver settings.
  ///
  /// @return 1 if the settings from the device match the cached copies, 0 if
  /// they do not.
  bool verifySettings()
  {
    // Bit 10 in TORQUE is write-only and will always read as 0.
    return driver.readReg(HPSDRegAddr::CTRL)   == ctrl   &&
           driver.readReg(HPSDRegAddr::TORQUE) == (torque & ~(1 << 10)) &&
           driver.readReg(HPSDRegAddr::OFF)    == off    &&
           driver.readReg(HPSDRegAddr::BLANK)  == blank  &&
           driver.readReg(HPSDRegAddr::DECAY)  == decay  &&
           driver.readReg(HPSDRegAddr::STALL)  == stall  &&
           driver.readReg(HPSDRegAddr::DRIVE)  == drive;
  }

  /// Re-writes the cached settings stored in this class to the device.
  ///
  /// You should not normally need to call this function because settings are
  /// written to the device whenever they are changed.  However, if
  /// verifySettings() returns false (due to a power interruption, for
  /// instance), then you could use applySettings() to get the device's settings
  /// back into the desired state.
  void applySettings()
  {
    writeTORQUE();
    writeOFF();
    writeBLANK();
    writeDECAY();
    writeDRIVE();
    writeSTALL();

    // CTRL is written last because it contains the ENBL bit, and we want to try
    // to have all the other settings correct first.  (For example, TORQUE
    // defaults to 0xFF (the maximum value), so it would be better to set a more
    // appropriate value if necessary before enabling the motor.)
    writeCTRL();
  }

  /// Enables the driver (ENBL = 1).
  void enableDriver()
  {
    ctrl |= (1 << 0);
    writeCTRL();
  }

  /// Disables the driver (ENBL = 0).
  void disableDriver()
  {
    ctrl &= ~(1 << 0);
    writeCTRL();
  }

  /// Sets the motor direction (RDIR).
  ///
  /// Allowed values are 0 or 1.
  ///
  /// You can use this command to control the direction of the stepper motor and
  /// leave the DIR pin disconnected.
  void setDirection(bool value)
  {
    if (value)
    {
      ctrl |= (1 << 1);
    }
    else
    {
      ctrl &= ~(1 << 1);
    }
    writeCTRL();
  }

  /// Returns the cached value of the motor direction (RDIR).
  ///
  /// This does not perform any SPI communication with the driver.
  bool getDirection()
  {
    return ctrl >> 1 & 1;
  }

  /// Advances the indexer by one step (RSTEP = 1).
  ///
  /// You can use this command to step the stepper motor and leave the STEP pin
  /// disconnected.
  ///
  /// The driver automatically clears the RSTEP bit after it is written.
  void step()
  {
    driver.writeReg(HPSDRegAddr::CTRL, ctrl | (1 << 2));
  }

  /// Sets the driver's stepping mode (MODE).
  ///
  /// This affects many things about the performance of the motor, including how
  /// much the output moves for each step taken and how much current flows
  /// through the coils in each stepping position.
  ///
  /// If an invalid stepping mode is passed to this function, then it selects
  /// 1/4 micro-step, which is the driver's default.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setStepMode(HPSDStepMode::MicroStep32);
  /// ~~~
  void setStepMode(HPSDStepMode mode)
  {
    // Pick 1/4 micro-step by default.
    uint8_t sm = 0b0010;

    switch (mode)
    {
    case HPSDStepMode::MicroStep1:   sm = 0b0000; break;
    case HPSDStepMode::MicroStep2:   sm = 0b0001; break;
    case HPSDStepMode::MicroStep4:   sm = 0b0010; break;
    case HPSDStepMode::MicroStep8:   sm = 0b0011; break;
    case HPSDStepMode::MicroStep16:  sm = 0b0100; break;
    case HPSDStepMode::MicroStep32:  sm = 0b0101; break;
    case HPSDStepMode::MicroStep64:  sm = 0b0110; break;
    case HPSDStepMode::MicroStep128: sm = 0b0111; break;
    case HPSDStepMode::MicroStep256: sm = 0b1000; break;
    }

    ctrl = (ctrl & 0b111110000111) | (sm << 3);
    writeCTRL();
  }

  /// Sets the driver's stepping mode (MODE).
  ///
  /// This version of the function allows you to express the requested
  /// microstepping ratio as a number directly.
  ///
  /// ~~~{.cpp}
  /// sd.setStepMode(32);
  /// ~~~
  ///
  /// \see setStepMode(HPSDStepMode)
  void setStepMode(uint16_t mode)
  {
    setStepMode((HPSDStepMode)mode);
  }

  /// Sets the current limit for a High-Power Stepper Motor Driver 36v4.
  ///
  /// The argument to this function should be the desired current limit in
  /// milliamps.
  ///
  /// WARNING: The 36v4 can supply up to about 4 A per coil continuously;
  /// higher currents might be sustainable for short periods, but can eventually
  /// cause the MOSFETs to overheat, which could damage them.  See the driver's
  /// product page for more information.
  ///
  /// This function allows you to set a current limit of up to 8 A (8000 mA),
  /// but we strongly recommend against using a current limit higher than 4 A
  /// (4000 mA) unless you are careful to monitor the MOSFETs' temperatures
  /// and/or restrict how long the driver uses the higher current limit.
  ///
  /// This function takes care of setting appropriate values for ISGAIN and
  /// TORQUE to get the desired current limit.
  void setCurrentMilliamps36v4(uint16_t current)
  {
    if (current > 8000) { current = 8000; }

    // From the DRV8711 datasheet, section 7.3.4, equation 2:
    //
    //   Ifs = (2.75 V * TORQUE) / (256 * ISGAIN * Risense)
    //
    // Rearranged:
    //
    //   TORQUE = (256 * ISGAIN * Risense * Ifs) / 2.75 V
    //
    // The 36v4 has an Risense of 30 milliohms, and "current" is in milliamps,
    // so:
    //
    //   TORQUE = (256 * ISGAIN * (30/1000) ohms * (current/1000) A) / 2.75 V
    //          = (7680 * ISGAIN * current) / 2750000
    //
    // We want to pick the highest gain (5, 10, 20, or 40) that will not
    // overflow TORQUE (8 bits, 0xFF max), so we start with a gain of 40 and
    // calculate the TORQUE value needed.
    uint8_t isgainBits = 0b11;
    uint16_t torqueBits = ((uint32_t)768  * current) / 6875;

    // Halve the gain and TORQUE until the TORQUE value fits in 8 bits.
    while (torqueBits > 0xFF)
    {
      isgainBits--;
      torqueBits >>= 1;
    }

    ctrl = (ctrl & 0b110011111111) | (isgainBits << 8);
    writeCTRL();
    torque = (torque & 0b111100000000) | torqueBits;
    writeTORQUE();
  }

  /// Sets the driver's decay mode (DECMOD).
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setDecayMode(HPSDDecayMode::AutoMixed);
  /// ~~~
  void setDecayMode(HPSDDecayMode mode)
  {
    decay = (decay & 0b00011111111) | (((uint8_t)mode & 0b111) << 8);
    writeDECAY();
  }

  /// Reads the status of the driver (STATUS register).
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// status condition (the upper 4 bits of the 12-bit STATUS register are not
  /// used).  You can simply compare the return value to 0 to see if any of the
  /// status bits are set, or you can use the logical AND operator (`&`) and the
  /// #HPSDStatusBit enum to check individual bits.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// if (sd.readStatus() & (1 << (uint8_t)HPSDStatusBit::UVLO))
  /// {
  ///   // Undervoltage lockout is active.
  /// }
  /// ~~~
  uint8_t readStatus()
  {
    return driver.readReg(HPSDRegAddr::STATUS);
  }

  /// Clears any status conditions that are currently latched in the driver.
  ///
  /// WARNING: Calling this function clears latched faults, which might allow
  /// the motor driver outputs to reactivate.  If you do this repeatedly without
  /// fixing an abnormal condition (like a short circuit), you might damage the
  /// driver.
  void clearStatus()
  {
    driver.writeReg(HPSDRegAddr::STATUS, 0);
  }

  /// Reads fault conditions indicated by the driver.
  ///
  /// The return value is the same as that which would be returned by
  /// readStatus(), except it only contains bits that indicate faults (STATUS
  /// bits 5:0).
  uint8_t readFaults()
  {
    return readStatus() & 0b00111111;
  }

  /// Clears any fault conditions that are currently latched in the driver.
  ///
  /// This function behaves the same as clearStatus(), except it only clears
  /// bits that indicate faults (STATUS bits 5:0).
  ///
  /// WARNING: Calling this function clears latched faults, which might allow
  /// the motor driver outputs to reactivate.  If you do this repeatedly without
  /// fixing an abnormal condition (like a short circuit), you might damage the
  /// driver.
  void clearFaults()
  {
    driver.writeReg(HPSDRegAddr::STATUS, ~0b00111111);
  }

protected:

  uint16_t ctrl, torque, off, blank, decay, stall, drive;

  /// Writes the cached value of the CTRL register to the device.
  void writeCTRL()
  {
    driver.writeReg(HPSDRegAddr::CTRL, ctrl);
  }

  /// Writes the cached value of the TORQUE register to the device.
  void writeTORQUE()
  {
    driver.writeReg(HPSDRegAddr::TORQUE, torque);
  }

  /// Writes the cached value of the OFF register to the device.
  void writeOFF()
  {
    driver.writeReg(HPSDRegAddr::OFF, off);
  }

  /// Writes the cached value of the BLANK register to the device.
  void writeBLANK()
  {
    driver.writeReg(HPSDRegAddr::BLANK, blank);
  }

  /// Writes the cached value of the DECAY register to the device.
  void writeDECAY()
  {
    driver.writeReg(HPSDRegAddr::DECAY, decay);
  }

  /// Writes the cached value of the STALL register to the device.
  void writeSTALL()
  {
    driver.writeReg(HPSDRegAddr::STALL, stall);
  }

  /// Writes the cached value of the DRIVE register to the device.
  void writeDRIVE()
  {
    driver.writeReg(HPSDRegAddr::DRIVE, drive);
  }

public:
  /// This object handles all the communication with the DRV8711.  Generally,
  /// you should not need to use it in your code for basic usage of a
  /// High-Power Stepper Motor Driver, but you might want to use it to access
  /// more advanced settings that the HighPowerStepperDriver class does not
  /// provide functions for.
  DRV8711SPI driver;
};
