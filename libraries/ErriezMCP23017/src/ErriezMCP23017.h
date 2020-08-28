/*
 * MIT License
 *
 * Copyright (c) 2020 Erriez
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*!
 * \file ErriezMCP23017.h
 * \brief MCP23017 I2C IO expander library for Arduino
 * \details
 *      Source:         https://github.com/Erriez/ErriezMCP23017
 *      Documentation:  https://erriez.github.io/ErriezMCP23017
 *
 *   ---------------------------------------------------------------------------
 *   Design notes:
 *   ---------------------------------------------------------------------------
 *   1 This library is designed for MCP23017 with I2C interface.
 *
 *   2 This library does not support the MCP23S017 with SPI interface.
 *     Workaround:
 *        None, use another library, or add SPI support to this library.
 *
 *   3 The INTB is not enabled in this library, because. INTA and INTB
 *     interrupts are ORed to INTA with configuration bit MIRROR=1 in IOCON
 *     register.
 *     Workaround:
 *        The application shall only use INTA.
 *
 *   4 Port, direction and pull-up states are cached in variables for speed:
 *     No additional register reads are needed.
 *
 *   ---------------------------------------------------------------------------
 *   MCP23017 limitations:
 *   ---------------------------------------------------------------------------
 *   1 The MCP23017 does not support rising or falling edge interrupts.
 *     Workaround:
 *        None: The MCP23017 support only the following interrupts:
 *        - Pin change (Recommended: Generates interrupt on pin level change)
 *        - Level high (Generates continues interrupts when pin is high)
 *        - Level low  (Generates continues interrupts when pin is low)
 *
 *   2 Note: The register IODIR bits are reversed:
 *       0: Output
 *       1: Input
 *     Workaround:
 *        Be careful with interpreting register IODIR.
 *
 *   ---------------------------------------------------------------------------
 *   MCP23017 major bugs:
 *   ---------------------------------------------------------------------------
 *   1 The INTA pin is released when reading from register GPIO or INTCAP.
 *     This happens when the application calls function pinRead() or
 *     portRead(). This is a documented chip limitation.
 *     Workaround:
 *        None.
 *
 *   2 Register INTF captures only one pin change. The MCP23017 does not
 *     update register INTF when multiple interrupts occurs at the same time.
 *     Workaround:
 *        1. The application shall read the INTF register with function
 *           getPortIntertuptStatus() as fast as possible after INTA is
 *           asserted.
 *           otherwise multiple pin interrupts are lost.
 *        2. The application shall poll pin INTA and read GPIO pin manually to
 *           detect pin changes.
 */

#ifndef ERRIEZ_MCP23017_H_
#define ERRIEZ_MCP23017_H_

#include <Arduino.h>
#include <Wire.h>

// Default configuration macro's
#define MCP23017_I2C_ADDRESS    0x20    //!< Default MCP23017 I2C address

// Register defines
#define MCP23017_REG_IODIR      0x00    //!< Controls the direction of the data I/O for port A.
#define MCP23017_REG_IPOL       0x02    //!< Configures the polarity on the corresponding GPIO port bits for port A.
#define MCP23017_REG_GPINTEN    0x04    //!< Controls the interrupt-on-change for each pin of port A.
#define MCP23017_REG_DEFVAL     0x06    //!< Controls the default comparaison value for interrupt-on-change for port A.
#define MCP23017_REG_INTCON     0x08    //!< Controls how the associated pin value is compared for the interrupt-on-change for port A.
#define MCP23017_REG_IOCON      0x0A    //!< Configuration register A
#define MCP23017_REG_GPPU       0x0C    //!< Controls the pull-up resistors for the port A pins.
#define MCP23017_REG_INTF       0x0E    //!< Reflects the interrupt condition on the port A pins.
#define MCP23017_REG_INTCAP     0x10    //!< Captures the port A value at the time the interrupt occured.
#define MCP23017_REG_GPIO       0x12    //!< Reflects the value on the port A.
#define MCP23017_REG_OLAT       0x14    //!< Provides access to the port A output latches.

#define MCP23017_NUM_REGS       0x16    //!< Total number of registers
#define MCP23017_NUM_PINS       16      //!< Total number of pins port A + B

// Bit masks
#define MCP23017_MASK_ALL_PINS  0xFFFF  //!< All 16-pins mask
#define MCP23017_MASK_REG_A     0x1E    //!< Address mask to select A registers on even addresses

// Bit defines IOCON register
#define IOCON_BANK              7       //!< Controls how the registers are addressed
#define IOCON_MIRROR            6       //!< INT Pins Mirror bit
#define IOCON_SEQOP             5       //!< Sequential Operation mode bit
#define IOCON_DISSLW            4       //!< Slew Rate control bit for SDA output
#define IOCON_ODR               2       //!< Configures the INT pin as an open-drain output
#define IOCON_INTPOL            1       //!< This bit sets the polarity of the INT output pin

//! Default MCP23017 configuration
#define REG_IOCON_VALUE \
    ((0<<IOCON_BANK) |      /* 1 = The registers associated with each port are separated into different banks. */ \
     (1<<IOCON_MIRROR) |    /* 1 = INTA and INTB pins are OR'ed to INTA (INTB disabled) */ \
     (0<<IOCON_SEQOP) |     /* 1 = Sequential operation disabled, address  pointer does not increment. */ \
     (0<<IOCON_DISSLW) |    /* 1 = Slew rate disabled */ \
     (0<<IOCON_ODR) |       /* 1 = Open-drain output (overrides the INTPOL bit.) */ \
     (0<<IOCON_INTPOL))     /* 1 = Active-high */

/*!
 * \brief Erriez MCP23017 I2C IO-Expander class
 */
class ErriezMCP23017
{
public:
    ErriezMCP23017(uint8_t i2cAddress=MCP23017_I2C_ADDRESS,
                   TwoWire *twoWire = &Wire);

    // Initialization
    bool begin(bool reset=true);

    // Single pin control
    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, uint8_t level);
    int digitalRead(uint8_t pin);

    // Set/get port direction input/output
    void setPortDirection(uint16_t outputPins);
    uint16_t getPortDirection();

    // Set/get port pull-up
    void setPortPullup(uint16_t pullupPins);
    uint16_t getPortPullup();

    // Pin read/write/toggle
    void pinWrite(uint8_t pin, bool level);
    void pinToggle(uint8_t pin);
    bool pinRead(uint8_t pin);

    // Port read/write/toggle/mask
    void portWrite(uint16_t value);
    void portToggle(uint16_t value);
    void portMask(uint16_t maskSet, uint16_t maskClear);
    uint16_t portRead();

    // Set/get interrupt mask/status
    void setInterruptPolarityINTA(bool activeHigh);
    uint16_t getPortInterruptMask();
    void setPortInterruptEnable(uint16_t pins);
    void setPortInterruptDisable(uint16_t pins);
    // Registers INTF and INTCAP can hold only one pin interrupt. A second
    // incoming pin interrupt is not updated in these registers and will be
    // lost. For this reason, reading these registers is useless and therefore
    // not implemented.

    // Interrupt generated by INTA pin
    bool interruptINTA();

    // Register access
    uint16_t registerRead(uint8_t reg);
    void registerWrite(uint8_t reg, uint16_t value);

    // I2C transfer status
    uint8_t getI2CStatus();

    // Debug function
    void dumpRegisters(HardwareSerial *serial);

    //! Port state since last portRead() call.
    uint16_t portState;

    //! Pins change on interrupt enabled pins since last intPinChanged() call.
    uint16_t pinsChanged;

    //! Falling edge on interrupt enabled pins since last intPinChanged() call.
    uint16_t pinsFalling;

    //! Rising edge on interrupt eanbled pins since last intPinChanged() call.
    uint16_t pinsRising;

private:
    TwoWire *_wire;             //!< Pointer to Wire
    uint8_t _i2cAddress;        //!< I2C device address
    uint8_t _i2cStatus;         //!< I2C status of the last I2C write

    // Cached variables to speed-up I2C bus access without I2C reads
    uint16_t _gpioPort;         //!< GPIO port status since last portRead() call
    uint16_t _gpioDir;          //!< GPIO port direction since last setPortDirection() call
    uint16_t _gpioPullup;       //!< GPIO port enabled pullups since last setPortPullup() call
    uint16_t _portStateLast;    //!< GPIO port status for pin change interrupt handling
};

#endif // ERRIEZ_MCP23017_H_
