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
 *      This example demonstrates the "Arduino easy" way to handle inputs and
 *      outputs. Handling inputs and outputs simultanesouly is not possible,
 *      because this example uses a blocking delay(). Please refer to PinPort
 *      example to handle inputs and outputs currently.
 *
 *      Hardware:
 *      - MCP23017 SDA, SCL connected to MCU I2C (TWI) pins
 *      - PA0..PA7: Button input with pull-up
 *      - PB0..PB7: LED output
 */

#include <Arduino.h>
#include <Wire.h>
#include <ErriezMCP23017.h>

// Default I2C Address 0x20
#define MCP23017_I2C_ADDRESS    0x20

// Create MCP23017 object
ErriezMCP23017 mcp = ErriezMCP23017(MCP23017_I2C_ADDRESS);


static void handleOutputs()
{
    Serial.println(F("Shift output PORTB..."));

    // Pins B0..B7 HIGH with a blocking delay
    for (uint8_t outputPin = 8; outputPin <= 15; outputPin++) {
       mcp.digitalWrite(outputPin, HIGH);
       delay(100);
    }
    // Pins B0..B7 LOW with a blocking delay
    for (uint8_t outputPin = 15; outputPin >= 7; outputPin--) {
       mcp.digitalWrite(outputPin, LOW);
       delay(100);
    }
}

static void handleInputs()
{
    Serial.println(F("Input PORTA:"));

    // Print state input pins PORTA
    for (uint8_t inputPin = 0; inputPin <= 7; inputPin++) {
        Serial.print(F("  A"));
        Serial.print(inputPin);
        Serial.print(F(": "));
        if (mcp.digitalRead(inputPin)) {
            Serial.println(F("HIGH"));
        } else {
            Serial.println(F("LOW"));
        }
    }
}

void setup()
{
    // Initialize Serial
    Serial.begin(115200);
    Serial.println(F("\nErriez MCP23017 I2C IO-expander Multiple Pin example"));

    // Initialize Wire
    Wire.begin();
    Wire.setClock(400000);

    // Initialize MCP23017
    while (!mcp.begin()) {
        Serial.print(F("Error: MCP23017 not found, code: "));
        Serial.println(mcp.getI2CStatus());
        delay(3000);
    }

    // Pins A0..A7 input with pull-up enabled
    for (uint8_t inputPin = 0; inputPin <= 7; inputPin++) {
       mcp.pinMode(inputPin, INPUT_PULLUP);
    }

    // Pins B0..B7 output
    for (uint8_t outputPin = 8; outputPin <= 15; outputPin++) {
       mcp.pinMode(outputPin, OUTPUT);
    }

    // Print registers
    // mcp.dumpRegisters(&Serial);
}

void loop()
{
    // Synchronous call to handle outputs
    handleOutputs();

    // Handle inputs
    handleInputs();
}
