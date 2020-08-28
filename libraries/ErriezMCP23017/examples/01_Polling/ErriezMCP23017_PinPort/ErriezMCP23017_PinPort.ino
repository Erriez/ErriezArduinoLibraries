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
 * \brief MCP23017 I2C 16-pin IO-expander example for Arduino
 * \details
 *      Source:         https://github.com/Erriez/ErriezMCP23017
 *      Documentation:  https://erriez.github.io/ErriezMCP23017
 *
 *      This example demonstrates:
 *      - Pin and port control.
 *      - Asynchronous handling of input and output port.
 *
 *      Hardware:
 *      - MCP23017 SDA, SCL connected to MCU I2C (TWI) pins
 *      - PA0..PA7: Input with pull-up
 *      - PB0..PB7: Output
 */

#include <Arduino.h>
#include <Wire.h>
#include <ErriezMCP23017.h>

// Port direction (Pin 1 = output, 0 is input)
#define PORT_DIRECTION          0xFF00
// Output pins
#define OUTPUT_PINS             PORT_DIRECTION
// Input pins is inverse of output pins
#define INPUT_PINS              (~(OUTPUT_PINS) & 0xFFFF)
// Initial port state
#define PORT_STATE              0xFF00

// LED pin defines
#define LED_PIN                 8   // First LED on B0
#define NUM_LEDS                8   // 8 LED's on PORTB

// Shift LED interval in milliseconds
#define SHIFT_INTERVAL_MS       100

// Default I2C Address 0x20
#define MCP23017_I2C_ADDRESS    0x20

// Create MCP23017 object
ErriezMCP23017 mcp = ErriezMCP23017(MCP23017_I2C_ADDRESS);


void handleOutputs()
{
    // static: Remember variable (put on stack)
    static uint8_t led = 0;
    static unsigned long tLast;
    unsigned long now;
    uint16_t pinsHigh;
    uint16_t pinsLow;

    // Shift LED at fixed interval
    now = millis();
    if ((now - tLast) > SHIFT_INTERVAL_MS) {
        tLast = now;

        // Shift LED
        led++;
        if (led >= NUM_LEDS) {
            led = 0;
        }

        // Set LEDs on PORTB
        pinsLow = OUTPUT_PINS;
        pinsHigh = (1<<(LED_PIN + led));
        mcp.portMask(pinsHigh, pinsLow);
    }
}

void handleInputs()
{
    // static: Remember variable (put on stack)
    static uint16_t inputPortLast;
    uint16_t inputPort;
    char line[20];

    // Read port state
    inputPort = mcp.portRead() & INPUT_PINS;

    // Check if port changed
    if (inputPortLast != inputPort) {
        // Store current port state
        inputPortLast = inputPort;

        // Print current port state
        snprintf_P(line, sizeof(line), PSTR("Port state:  0x%04x"), inputPort);
        Serial.println(line);
    }
}

void setup()
{
    uint16_t setPins;
    uint16_t clearPins;

    // Initialize Serial
    Serial.begin(115200);
    Serial.println(F("\nErriez MCP23017 I2C IO-expander Pin/Port example"));

    // Initialize Wire
    Wire.begin();
    Wire.setClock(400000);

    // Initialize MCP23017
    while (!mcp.begin()) {
        Serial.print(F("Error: MCP23017 not found, code: "));
        Serial.println(mcp.getI2CStatus());
        delay(3000);
    }

    // -----------------------------------------------------------------
    // Test single pin
    // -----------------------------------------------------------------
    // Configure single pin A0 output
    mcp.pinMode(0, OUTPUT);

    // Write/read pin B0
    Serial.print(F("Pin B0 write HIGH: "));
    mcp.pinWrite(8, HIGH);
    Serial.println(mcp.pinRead(0));
    delay(500);
    mcp.pinWrite(8, LOW);
    Serial.print(F("Pin B0 write LOW: "));
    Serial.println(mcp.pinRead(8));
    delay(500);

    // Toggle/read pin B0
    mcp.pinToggle(8);
    Serial.print(F("Pin B0 toggle: "));
    Serial.println(mcp.pinRead(8));
    delay(500);

    // Configure single pin A0 input without pull-up
    mcp.pinMode(0, INPUT);
    Serial.print(F("Pin A0 input: "));
    Serial.println(mcp.pinRead(0));
    delay(500);

    // Configure single pin input with pull-up
    mcp.pinMode(0, INPUT_PULLUP);
    Serial.print(F("Pin A0 input pull-up: "));
    Serial.println(mcp.pinRead(0));
    delay(1000);

    // -----------------------------------------------------------------
    // Test port
    // -----------------------------------------------------------------
    // Set/get port modes
    mcp.setPortDirection(PORT_DIRECTION);
    Serial.print(F("Port output: 0x"));
    Serial.println(mcp.getPortDirection(), HEX);

    // Enable/read port pullups
    mcp.setPortPullup(~(OUTPUT_PINS) & 0xFFFF);
    Serial.print(F("Port pullup: 0x"));
    Serial.println(mcp.getPortDirection(), HEX);

    // Write/read port
    mcp.portWrite(PORT_STATE);
    Serial.print(F("Port state:  0x"));
    Serial.println(mcp.portRead(), HEX);

    // Toggle/read port state
    mcp.portToggle(OUTPUT_PINS);
    Serial.print(F("Port state:  0x"));
    Serial.println(mcp.portRead(), HEX);

    // Mask/read port state
    setPins = 0xF000;
    clearPins = 0x00F0;
    mcp.portMask(setPins, clearPins);
    Serial.print(F("Port state:  0x"));
    Serial.println(mcp.portRead(), HEX);

    // -----------------------------------------------------------------
    // Port input pullup
    // -----------------------------------------------------------------
    // Change port to input pull-up
    // Set/get port modes
    mcp.setPortDirection(PORT_DIRECTION);
    Serial.print(F("Port output: 0x"));
    Serial.println(mcp.getPortDirection(), HEX);

    // Enable/read port pullups
    mcp.setPortPullup(INPUT_PINS);
    Serial.print(F("Port pullup: 0x"));
    Serial.println(mcp.getPortDirection(), HEX);

    // -----------------------------------------------------------------
    // End setup
    // -----------------------------------------------------------------
    // Dump registers
    // mcp.dumpRegisters(&Serial);

    Serial.print(F("Polling port..."));
}

void loop()
{
    // Handle outputs
    handleOutputs();

    // Handle inputs
    handleInputs();
}
