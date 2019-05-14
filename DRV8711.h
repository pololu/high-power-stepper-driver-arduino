// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/

/*! \file DRV8711.h
 *
 * This is the main header file for the DRV8711 library, a library for
 * controllering the DRV8711 micro-stepping stepper motor driver.
 *
 * For more information about this library, see:
 *
 *   https://github.com/pololu/drv8711-arduino
 *
 * That is the main repository for this library.
 *
 */

#pragma once

#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>

/*! This class provides low-level functions for reading and writing from the SPI
 * interface of an DRV8711 micro-stepping stepper motor driver.
 *
 * Most users should use the DRV8711 class, which provides a higher-level
 * interface, instead of this class. */
class DRV8711SPI
{
public:
    /*! Configures this object to use the specified pin as a slave select pin.
     * You must use a slave select pin; the DRV8711 requires it. */
    void init(uint8_t slaveSelectPin) {
        ssPin = slaveSelectPin;
        digitalWrite(ssPin, LOW);
        pinMode(ssPin, OUTPUT);
    }

    /*! Reads the register at the given address and returns its raw value. */
    uint16_t readReg(uint8_t address)
    {
        selectChip();

        uint16_t dataOut = transfer((0x8 | address) << 12);
        deselectChip();
        // Mask off read/write bit
        dataOut = dataOut & 0x0FFF;
        return dataOut;
    }

    /*! Writes the specified value to a register. */
    void writeReg(uint8_t address, uint16_t value)
    {
        selectChip();
        transfer((address << 12) | value);

        // The CS line must go low after writing for the value to actually take
        // effect.
        deselectChip();
    }

private:

    SPISettings settings = SPISettings(500000, MSBFIRST, SPI_MODE0);

    uint16_t transfer(uint16_t value)
    {
        return SPI.transfer16(value);
    }

    void selectChip()
    {
        digitalWrite(ssPin, HIGH);
        SPI.beginTransaction(settings);
    }

    void deselectChip()
    {
       SPI.endTransaction();
       digitalWrite(ssPin, LOW);
    }

    uint8_t ssPin;
};

/*! This class provides high-level functions for controlling an DRV8711
 *  micro-stepping motor driver.
 *
 * It provides access to all the features of the DRV8711 SPI interface
 * except the watchdog timer. */
class DRV8711
{
public:
    /*! The default constructor. */
    DRV8711()
    {
        // All settings set to default configurations on power-reset
        ctrl = 0xC10;
        torque = 0x1FF;
        off = 0x30;
        blank = 0x80;
        decay = 0x110;
        stall = 0x40;
        drive = 0xA59;
        status = 0x0;
    }

    /*! Addresses of control and status registers. */
    enum regAddr
    {
        CTRL = 0x00,
        TORQUE = 0x01,
        OFF = 0x02,
        BLANK = 0x03,
        DECAY = 0x04,
        STALL = 0x05,
        DRIVE = 0x06,
        STATUS = 0x07,
    };

  	/*! Possible arguments to setGain(). */
  	enum isgain
  	{
  		Gain5 = 5,
  		Gain10 = 10,
  		Gain20 = 20,
  		Gain40 = 40,
  	};

  	/*! Possible arguments to setDeadTime(). */
  	enum deadTime
  	{
  		DeadTime400ns = 400,
  		DeadTime450ns = 450,
  		DeadTime650ns = 650,
  		DeadTime850ns = 850,
  	};

    /*! Possible arguments to setStepMode(). */
    enum stepMode
    {
        MicroStep256 = 256,
        MicroStep128 = 128,
        MicroStep64 = 64,
        MicroStep32 = 32,
        MicroStep16 = 16,
        MicroStep8 = 8,
        MicroStep4 = 4,
        MicroStep2 = 2,
        MicroStep1 = 1,
    };

    /*! Possible arguments to setDecayMode(). */
    enum setDecayMode
    {
      Slow = 0b000,
      SlowIncreaseMixedDecrease = 0b001,
      Fast = 0b010,
      Mixed = 0b011,
      SlowIncreaseAutoMixedDecrease = 0b100,
      AutoMixed = 0b101,
    };

    /*! Possible arguments to checkErrorStatus(). */
    enum errorType
    {
      overTemp = 0b000,
      aOverCurrent = 0b001,
      bOverCurrent = 0b010,
      aPreDriver = 0b011,
      bPreDriver = 0b100,
      underVoltageLockout = 0b101,
      stallDetected = 0b110,
      latchedStallDetected = 0b111,
    };

    /*! Configures this object to use the specified pin as a slave select pin.
     * You must use a slave select pin; the DRV8711 requires it. */
    void init(uint8_t slaveSelectPin)
    {
        driver.init(slaveSelectPin);
    }

    /*! Sets the ENBL bit to 1, enabling the driver. */
    void enableDriver()
    {
        ctrl |= 1;
        writeCTRL();
    }

    /*! Sets the ENBL bit to 0, disabling the driver. */
    void disableDriver()
    {
        ctrl &= 0;
        writeCTRL();
    }

	/*! Sets the RDIR bit: 0 sets direction by DIR pin and 1 sets direction
	 * inverse of DIR pin. */
	void flipDirection()
	{
		ctrl = ctrl ^ (1 << 1);
		writeCTRL();
	}

	/*! Sets the RSTEP bit to 1: indexer will advance one step, automatically
	 * cleared after write. */
	void step()
	{
		ctrl |= (1 << 2);
    writeCTRL();
	}

    /*! Configures the driver to have the specified stepping mode.
     *
     * This affects many things about the performance of the motor, including
     * how much the output moves for each step taken and how much current flows
     * through the coils in each stepping position.
     *
     * The argument to this function should be one of the members of the
     * #stepMode enum.
     *
     * If an invalid stepping mode is passed to this function, then it selects
     * 1/4 micro-step, which is the driver's default. */
    void setStepMode(uint16_t mode)
    {
        // Pick 1/4 micro-step by default.
        uint16_t sm = 0b0010;

        // The order of these cases matches the order in Table 3 of the
        // DRV8711 datasheet.
        switch(mode)
        {
        case MicroStep1: sm = 0b0000; break;
        case MicroStep2: sm = 0b0001; break;
        case MicroStep4: sm = 0b0010; break;
        case MicroStep8: sm = 0b0011; break;
        case MicroStep16: sm = 0b0100; break;
        case MicroStep32: sm = 0b0101; break;
        case MicroStep64: sm = 0b0110; break;
        case MicroStep128: sm = 0b0111; break;
        case MicroStep256: sm = 0b1000; break;
        }

        ctrl = (ctrl & 0b111110000111) | (sm << 3);
        writeCTRL();
    }

	/*! Sets EXSTALL bit to 0, detecting stall externally.
	 * By default, EXSTALL bit set to 0: internal stall detect. */
	void setExternalStallDetection()
	{
		ctrl |= (1 << 7);
		writeCTRL();
	}

	/*! Sets EXSTALL bit to 1, detecting stall internally.
	 * By default, EXSTALL bit set to 0: internal stall detect. */
	void setInternalStallDetection()
	{
		ctrl &= ~(1 << 7);
		writeCTRL();
	}

	/*! Sets isgain bit to a gain of: 5, 10, 20, or 40. */
	void setGain(uint8_t gain)
	{
		// Pick gain of 20 by default.
		uint16_t ag = 0b10;

		switch(gain)
		{
		case Gain5: ag = 0b00; break;
		case Gain10: ag = 0b01; break;
		case Gain20: ag = 0b10; break;
		case Gain40: ag = 0b11; break;
		}

		ctrl = (ctrl & 0b110011111111) | (ag << 8);
		writeCTRL();
	}

	/*! Sets DTIME bit to time: 400 ns, 450 ns, 650 ns, or 850 ns. */
	void setDeadTime(uint8_t dTime)
	{
		// Pick dead time of 850 ns by default.
		uint16_t dt = 0b11;

		switch(dTime)
		{
		case DeadTime400ns: dt = 0b00; break;
		case DeadTime450ns: dt = 0b01; break;
		case DeadTime650ns: dt = 0b10; break;
		case DeadTime850ns: dt = 0b11; break;
		}

		ctrl = (ctrl & 0b001111111111) | (dt << 10);
		writeCTRL();
	}

  void reset()
  {
    ctrl = 0xC10;
    torque = 0x1FF;
    off = 0x30;
    blank = 0x80;
    decay = 0x110;
    stall = 0x40;
    drive = 0xA59;
    status = 0x0;
    while(!verifySettings()){
        driver.writeReg(CTRL, ctrl);
        driver.writeReg(TORQUE, torque);
        driver.writeReg(OFF, off);
        driver.writeReg(BLANK, blank);
        driver.writeReg(DECAY, decay);
        driver.writeReg(STALL, stall);
        driver.writeReg(DRIVE, drive);
        driver.writeReg(STATUS, status);
    }
  }

	/*! Sets TORQUE bits [7:0] in TORQUE register.  See equation in
	 * datasheet. */
	void setTorque(uint8_t torqueValue)
	{
		torque = (torque & 0b11100000000) | torqueValue;
		writeTORQUE();
	}

	/*! Sets TOFF bits [7:0] in 500 ns increments in OFF register.  See
	 * datasheet for more details. */
	void setOffTime(uint8_t offTime)
	{
		off = (off & 0b100000000) | offTime;
		writeOFF();
	}

	/*! Sets PWMMODE bit: 0 - uses internal indexer and 1 - bypasses indexer,
	 * using xINx inputs to control outputs. See datasheet for more
	 * information. */
	void setPWMMode(uint16_t pwmMode)
	{
		off = off ^ (1 << 8);
		writeOFF();
	}

  /* Sets current trip blanking time, in increments of 20 ns */
  void setBlankingTime(uint8_t tBlank)
  {
    blank = ((blank&0b100000000) | tBlank);
    writeBLANK();
  }

  /* Disable adaptive blanking time */
  void disableAdaptiveBlankingTime()
  {
    blank = (blank & 0b011111111);
    writeBLANK();
  }

  /* Enable adaptive blanking time */
  void enableAdaptiveBlankingTime()
  {
    blank = (blank  | 0b100000000);
    writeBLANK();
  }

  /* setDecayMode */
  void setDecayMode(uint8_t mode)
  {
    // Pick a default decay mode of slow decay for increasing currents, mixed decay for decreasing currents
    uint16_t dm = 0b001;

		switch(mode)
		{
		case Slow: dm = 0b000; break;
		case SlowIncreaseMixedDecrease: dm = 0b001; break;
		case Fast: dm = 0b010; break;
		case Mixed: dm = 0b011; break;
    case SlowIncreaseAutoMixedDecrease: dm = 0b100; break;
		case AutoMixed: dm = 0b101; break;
		}

    decay = (decay & 0b00011111111) | (dm << 8); // this might not work the way I think it does
    writeDECAY();
  }

  /* sets Decay transition Time in increments of 500 ns */
  void setDecayTransition(uint8_t time)
  {
    decay = (decay & 0b11100000000) | time;
    writeDECAY();
  }

  /* reads the CTRL register */
  uint16_t readCTRLReg()
  {
    return driver.readReg(CTRL);
  }

  /* reads the TORQUE register */
  uint16_t readTORQUEReg()
  {
    return driver.readReg(TORQUE);
  }

  /* reads the OFF register */
  uint16_t readOFFReg()
  {
    return driver.readReg(OFF);
  }

  /* reads the BLANK register */
  uint16_t readBLANKReg()
  {
    return driver.readReg(BLANK);
  }

  /* reads the DECAY register */
  uint16_t readDECAYReg()
  {
    return driver.readReg(DECAY);
  }


  /* reads the STALL register */
  uint16_t readSTALLReg()
  {
    return driver.readReg(STALL);
  }


  /* reads the DRIVE register */
  uint16_t readDRIVEReg()
  {
    return driver.readReg(DRIVE);
  }


  /* reads the STATUS register */
  uint16_t readSTATUSReg()
  {
    return driver.readReg(STATUS);
  }


  /* clears the status register */
  void clearStatusReg()
  {
    status = 0x00;
    writeSTATUS();
  }

  /* blanket is there any error function */
  bool errorDetected()
  {
    if((readSTATUSReg() & 0xFF) != 0)
      return true;
    return false;
  }

  /* sees if there is a specific error detected */
  bool errorDetected(uint8_t error)
  {
    status = readSTATUSReg();
    if(error < 8){
      return (status & (1 << error));
    }
    else return false;
  }

  bool verifySettings()
  {
    return driver.readReg(CTRL) == ctrl &&
           driver.readReg(TORQUE) == torque &&
           driver.readReg(OFF) == off &&
           driver.readReg(BLANK) == blank &&
           driver.readReg(DECAY) == decay &&
           driver.readReg(STALL) == stall &&
           driver.readReg(DRIVE) == drive &&
           driver.readReg(STATUS) == status;
  }

  void applySettings()
  {
    writeCTRL();
    writeTORQUE();
    writeOFF();
    writeBLANK();
    writeDECAY();
    writeSTATUS();
  }

protected:

    uint16_t ctrl, torque, off, blank, decay, status;

    /*! Writes the cached value of the CTRL register to the device. */
    void writeCTRL()
    {
        driver.writeReg(CTRL, ctrl);
    }

    /*! Writes the cached value of the TORQUE register to the device. */
    void writeTORQUE()
    {
        driver.writeReg(TORQUE, torque);
    }

    /*! Writes the cached value of the OFF register to the device. */
    void writeOFF()
    {
        driver.writeReg(OFF, off);
    }

    /*! Writes the cached value of the BLANK register to the device. */
    void writeBLANK()
    {
        driver.writeReg(BLANK, blank);
    }

    /*! Writes the cached value of the DECAY register to the device. */
    void writeDECAY()
    {
        driver.writeReg(DECAY, decay);
    }

    /*! Writes the cached value of the STALL register to the device. */
    void writeSTALL()
    {
        driver.writeReg(STALL, stall);
    }

    /*! Writes the cached value of the DRIVE register to the device. */
    void writeDRIVE()
    {
        driver.writeReg(DRIVE, drive);
    }

    /*! Writes the cached value of the STATUS register to the device. */
    void writeSTATUS()
    {
        driver.writeReg(STATUS, status);
    }

public:
    /*! This object handles all the communication with the DRV8711.  It is
     * only marked as public for the purpose of testing this library; you should
     * not use it in your code. */
    DRV8711SPI driver;
};
