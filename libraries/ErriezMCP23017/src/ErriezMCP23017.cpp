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

#include "ErriezMCP23017.h"
#include <Wire.h>


/*!
 * \brief ErriezMCP23017 Constructor
 * \details
 *      The constructor initializes internal variables and does not call I2C functions
 * \param i2cAddress
 *      7-bit MCP23017 I2C device address
 * \param twoWire
 *      Default is Wire object to select I2C bus.
 */
ErriezMCP23017::ErriezMCP23017(uint8_t i2cAddress, TwoWire *twoWire) :
    _wire(twoWire), _i2cAddress(i2cAddress), _i2cStatus(0),
    _gpioPort(0), _gpioDir(0), _gpioPullup(0)
{
    // Nothing to do
}

//------------------------------------------------------------------------------
/*!
 * \brief Initialize MCP23017
 * \param reset
 *      Perform resetting registers to default values (Default enabled)
 * \retval true
 *      MCP23017 detected
 * \retval false
 *      MCP23017 not detected
 */
bool ErriezMCP23017::begin(bool reset)
{
    // Write default configuration to IOCON register
    registerWrite(MCP23017_REG_IOCON,
                  (REG_IOCON_VALUE << 8) | REG_IOCON_VALUE);

    // Return I2C write status
    if (_i2cStatus != 0) {
        // I2C register write failed
        return false;
    }

    if (reset) {
        // Disable interrupt pins
        setPortInterruptDisable(MCP23017_MASK_ALL_PINS);
        // Interrupt pin change
        registerWrite(MCP23017_REG_INTCON, 0);
        // Write port
        portWrite(_gpioPort);
        // Set port pull-up
        setPortPullup(_gpioPullup);
        // Set port direction
        setPortDirection(_gpioDir);
    }

    // Read current port state and release INTA pin
    _portStateLast = portRead();

    return true;
}

/*!
 * \brief Set direction of a single pin
 * \param pin
 *      Pin number 0..15 (PORTA = 0..7, PORTB = 8..15)
 * \param mode
 *      OUTPUT: Configure pin as output\n
 *      INPUT: Configure pin as input\n
 *      INPUT_PULLUP: Configure pin with input pull-up
 */
void ErriezMCP23017::pinMode(uint8_t pin, uint8_t mode)
{
    // Set pin mode
    if (pin < MCP23017_NUM_PINS) {
        switch (mode) {
            case OUTPUT:
                // Pin to output
                _gpioDir |= (1<<pin);
                break;
            case INPUT:
                // Disable pull-up
                _gpioPullup &= ~(1<<pin);
                // Pin to input
                _gpioDir &= ~(1<<pin);
                break;
            case INPUT_PULLUP:
                // Enable pull-up
                _gpioPullup |= (1<<pin);
                // Pin to input
                _gpioDir &= ~(1<<pin);
                break;
        }
        // Set port pull-up
        setPortPullup(_gpioPullup);
        // Set port direction
        setPortDirection(_gpioDir);
    }
}

/*!
 * \brief Set state of a single pin
 * \param pin
 *      Pin number 0..15 (PORTA = 0..7, PORTB = 8..15)
 * \param state
 *      HIGH = 1
 *      LOW = 0
 */
void ErriezMCP23017::digitalWrite(uint8_t pin, uint8_t state)
{
    // Set pin level
    pinWrite(pin, (bool)state);
}

/*!
 * \brief Get state of a single pin
 * \param pin
 *      Pin number 0..15 (PORTA = 0..7, PORTB = 8..15)
 * \retval HIGH = 1
 * \retval LOW = 0
 */
int ErriezMCP23017::digitalRead(uint8_t pin)
{
    // Read pin level
    return (int)pinRead(pin);
}

//------------------------------------------------------------------------------
/*!
 * \brief Set PORT direction all pins
 * \param outputPins
 *      PORT direction pins 0..15, Arduino compatible:\n
 *          Bit value '0': INPUT\n
 *          Bit value '1': OUTPUT
 */
void ErriezMCP23017::setPortDirection(uint16_t outputPins)
{
    // Store port direction
    _gpioDir = outputPins;
    // Set port direction
    registerWrite(MCP23017_REG_IODIR, ~(_gpioDir));
}

/*!
 * \brief Get PORT direction all pins
 * \return
 *      PORT direction pins 0..15, Arduino compatible:\n
 *          Bit value '0': INPUT\n
 *          Bit value '1': OUTPUT
 */
uint16_t ErriezMCP23017::getPortDirection()
{
    // Get port direction
    // Inverse value, because IODIR register: Bit '0': OUTPUT, '1': INPUT
    return ~(registerRead(MCP23017_REG_IODIR));
}

/*!
 * \brief Set PORT pullup all pins
 * \param pullupPins
 *      Set PORT pull-up pins 0..15:\n
 *          Bit value '0': Pull-up unchanged\n
 *          Bit value '1': Pull-up enable
 */
void ErriezMCP23017::setPortPullup(uint16_t pullupPins)
{
    // Store pull-up
    _gpioPullup = pullupPins;
    // Set pull-up pins
    registerWrite(MCP23017_REG_GPPU, _gpioPullup);
}

/*!
 * \brief Get PORT pullup all pins
 * \return
 *      PORT pull-up pins 0..15:\n
 *          Bit value '0': Pull-up disabled\n
 *          Bit value '1': Pull-up enable
 */
uint16_t ErriezMCP23017::getPortPullup()
{
    // Get pull-up pins
    return registerRead(MCP23017_REG_GPPU);
}

//------------------------------------------------------------------------------
/*!
 * \brief Set pin state
 * \param pin
 *      Pin number 0..15 (PORTA = 0..7, PORTB = 8..15)
 * \param level
 *      HIGH = 1, LOW = 0
 */
void ErriezMCP23017::pinWrite(uint8_t pin, bool level)
{
    if (pin < MCP23017_NUM_PINS) {
        if (level == HIGH) {
            portWrite(_gpioPort | (1<<pin));
        } else {
            portWrite(_gpioPort & ~(1<<pin));
        }
    }
}

/*!
 * \brief Toggle state of a single pin (only for output pins)
 * \param pin
 *      Pin number 0..15 (PORTA = 0..7, PORTB = 8..15)
 */
void ErriezMCP23017::pinToggle(uint8_t pin)
{
    if (pin < MCP23017_NUM_PINS) {
        // Toggle pin
        portToggle(1<<pin);
    }
}

/*!
 * \brief Read state of a single pin (input and output pins)
 * \param pin
 *      Pin number 0..15 (PORTA = 0..7, PORTB = 8..15)
 * \retval HIGH = 1
 * \retval LOW = 0
 */
bool ErriezMCP23017::pinRead(uint8_t pin)
{
    bool pinLevel = LOW;

    if (pin < MCP23017_NUM_PINS) {
        if (portRead() & (1<<pin)) {
            pinLevel = HIGH;
        }
    }
    return pinLevel;
}

/*!
 * \brief Set all pin states
 * \param value
 *      16 pins, bit value '0' = LOW, '1' = HIGH
 */
void ErriezMCP23017::portWrite(uint16_t value)
{
    // Store pins
    _gpioPort = value;
    // Write GPIO pin levels
    registerWrite(MCP23017_REG_GPIO, _gpioPort);
}

/*!
 * \brief Toggle pin states (output pins only)
 * \param value
 *      16 pins, bit value '0' = unchanged, '1' = toggle
 */
void ErriezMCP23017::portToggle(uint16_t value)
{
    // Toggle and store pin levels
    _gpioPort ^= value;
    // Write GPIO pin levels
    registerWrite(MCP23017_REG_GPIO, _gpioPort);
}

/*!
 * \brief Clear and set pin states
 * \param maskSet
 *      Bit value '1': Pins to HIGH
 * \param maskClear
 *      Bit value '1': Pins to LOW
 */
void ErriezMCP23017::portMask(uint16_t maskSet, uint16_t maskClear)
{
    _gpioPort &= ~(maskClear);
    _gpioPort |= maskSet;
    registerWrite(MCP23017_REG_GPIO, _gpioPort);
}

/*!
 * \brief Read PORT of all pins (input and output pins)
 * \return
 *      State of all 16 pins
 */
uint16_t ErriezMCP23017::portRead()
{
    // NOTE: Bug in the MCP23017: Reading the GPIO register releases INT pin
    portState = registerRead(MCP23017_REG_GPIO);

    // Return port state
    return portState;
}

//------------------------------------------------------------------------------
/*!
 * \brief Set interrupt polarity INTA
 * \param activeHigh
 *      HIGH = 1: Active high, LOW = 0: Active low (default)
 */
void ErriezMCP23017::setInterruptPolarityINTA(bool activeHigh)
{
    uint16_t value;

    // Read configuration register
    value = registerRead(MCP23017_REG_IOCON);

    // Set polarity INT pin
    if (activeHigh) {
        // Active high
        value |= ((1<<IOCON_INTPOL) << 8) | (1<<IOCON_INTPOL);
    } else {
        // Active low [default]
        value &= ~((1<<IOCON_INTPOL) << 8) | (1<<IOCON_INTPOL);
    }

    // Write configuration register
    registerWrite(MCP23017_REG_IOCON, value);
}

/*!
 * \brief Get interrupt mask all pins
 * \return
 *      Interrupt enabled
 */
uint16_t ErriezMCP23017::getPortInterruptMask()
{
    return registerRead(MCP23017_REG_GPINTEN);
}

/*!
 * \brief Enable interrupt change on pins
 * \details
 *      The MCP23017 does not support edge interrupts. This is handled by software.
 * \param pins
 *      Pins to enable interrupt change
 */
void ErriezMCP23017::setPortInterruptEnable(uint16_t pins)
{
    uint16_t intenValue;

    // Input pins
    setPortDirection(_gpioDir & ~(pins));

    // Enable interrupt pins
    intenValue = registerRead(MCP23017_REG_GPINTEN);
    intenValue |= pins;
    registerWrite(MCP23017_REG_GPINTEN, intenValue);
}

/*!
 * \brief Disable interrupt on pins
 * \param pins
 *     Pins to disable
 */
void ErriezMCP23017::setPortInterruptDisable(uint16_t pins)
{
    uint16_t value;

    // Disable interrupt pins
    value = registerRead(MCP23017_REG_GPINTEN);
    value &= ~(pins);
    registerWrite(MCP23017_REG_GPINTEN, value);
}

/*!
 * \brief MCP23017 INTA pin changed
 * \details
 *      The application should call this function when the MCP23017 INTA pin changed.
 *      Default: Falling edge
 *      This function re
 * \retval true
 *      At least one pin changed
 * \retval false
 *      No pins changed (Pin pulse was too short, or INTA edge did not match)
 * \returns
 *          portState:    PORT state since last portRead() call\n
 *          pinsChanged:  Changed pins on interrupt pins since last call\n
 *          pinsFalling:  Falling edge on interrupt pins since last call\n
 *          pinsRising:   Rising edge on interrupt pins since last call
 */
bool ErriezMCP23017::interruptINTA()
{
    // Read interrupt flag register which holds only one pin change
    pinsChanged = registerRead(MCP23017_REG_INTF);

    // A read of GPIO registers clears INTF register releases MCP23017 INTA pin
    portState = portRead();

    // Calculate changed pins
    pinsChanged |= _portStateLast ^ portState;

    // Calculate pins falling
    pinsFalling = ~(portState) & pinsChanged;
    // Calculate pins rising
    pinsRising = portState & pinsChanged;

    // Check if pin changed, but missed previous edge
    if ((_portStateLast & pinsChanged) == (pinsChanged & portState)) {
        pinsFalling |= pinsChanged;
        pinsRising |= pinsChanged;
    }

    // Store current pin state
    _portStateLast = portState;

    if (pinsChanged) {
        return true;
    } else {
        return false;
    }
}

//------------------------------------------------------------------------------
/*!
 * \brief MCP23017 I2C read register
 * \param reg
 *      MCP23017 register
 * \return
 *      MCP23017 register value
 */
uint16_t ErriezMCP23017::registerRead(uint8_t reg)
{
    _wire->beginTransmission(_i2cAddress);
    _wire->write(reg & MCP23017_MASK_REG_A);
    _i2cStatus = _wire->endTransmission(false);

    // Read two 8-bit register values with auto address increment
    (void)_wire->requestFrom(_i2cAddress, (uint8_t)2);

    // Return I2C read buffer
    return _wire->read() | (_wire->read() << 8);
}

/*!
 * \brief MCP23017 I2C write register
 * \param reg
 *      MCP23017 register
 * \param value
 *      MCP23017 value
 */
void ErriezMCP23017::registerWrite(uint8_t reg, uint16_t value)
{
    // Write 16-bit value to two MCP23017 registers
    _wire->beginTransmission(_i2cAddress);
    _wire->write(reg & MCP23017_MASK_REG_A);
    _wire->write(value & 0xff);
    _wire->write(value >> 8);
    // Generate I2C stop and store transfer status
    _i2cStatus = _wire->endTransmission(true);
}

/*!
 * \brief Return status of the last I2C write, returned by Wire endTransfer()
 * \retval 0: Success
 * \retval 1: Data too long to fit in transmit buffer
 * \retval 2: Received NACK on transmit of address
 * \retval 3: Received NACK on transmit of data
 * \retval 4: Other error
 */
uint8_t ErriezMCP23017::getI2CStatus()
{
    /* Status of the endTransmission (I2C addressing, repeated start ignored):
     */
    return _i2cStatus;
}

/*!
 * \brief Print I2C registers on serial port
 * \details
 *     This function is optimized away by the compiler when not used
 * \param serial
 *     Serial port
 */
void ErriezMCP23017::dumpRegisters(HardwareSerial *serial)
{
    serial->println(F("MCP23017 registers:"));

    serial->print(F("  00 IODIR:   0x"));
    serial->println(registerRead(MCP23017_REG_IODIR), HEX);
    serial->print(F("  02 IPOL:    0x"));
    serial->println(registerRead(MCP23017_REG_IPOL), HEX);
    serial->print(F("  04 GPINTEN: 0x"));
    serial->println(registerRead(MCP23017_REG_GPINTEN), HEX);
    serial->print(F("  06 DEFVAL:  0x"));
    serial->println(registerRead(MCP23017_REG_DEFVAL), HEX);
    serial->print(F("  08 INTCON:  0x"));
    serial->println(registerRead(MCP23017_REG_INTCON), HEX);
    serial->print(F("  0A IOCON:   0x"));
    serial->println(registerRead(MCP23017_REG_IOCON), HEX);
    serial->print(F("  0C GPPU:    0x"));
    serial->println(registerRead(MCP23017_REG_GPPU), HEX);
    serial->print(F("  0E INTF:    0x"));
    serial->println(registerRead(MCP23017_REG_INTF), HEX);
    serial->print(F("  10 INTCAP:  0x"));
    serial->println(registerRead(MCP23017_REG_INTCAP), HEX);
    serial->print(F("  12 GPIO:    0x"));
    serial->println(registerRead(MCP23017_REG_GPIO), HEX);
    serial->print(F("  14 OLAT:    0x"));
    serial->println(registerRead(MCP23017_REG_OLAT), HEX);
}
