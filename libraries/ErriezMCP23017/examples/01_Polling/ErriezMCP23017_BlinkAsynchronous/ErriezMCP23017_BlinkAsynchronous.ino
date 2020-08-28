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
 *      This example blinks three LED's asynchronous independent from each other.
 *
 *      It uses the built-in timer to blink the LED without blocking the main
 *      application loop. This is a recommended design for real applications.
 *
 *      Hardware:
 *      - MCP23017 SDA, SCL connected to MCU I2C (TWI) pins
 *      - PB0..PB2: LED output pin
 */

#include <Arduino.h>
#include <Wire.h>
#include <ErriezMCP23017.h>

// PORTA: pins 0..7
// PORTB: pins 8..15
#define LED0_PIN                    8   // Pin B0

// Total number of LED's
#define NUM_LEDS                    3

// Default I2C Address 0x20
#define MCP23017_I2C_ADDRESS        0x20

// Blink interval for multiple LED's in milliseconds
const uint16_t ledBlinkInterval[NUM_LEDS] = {
    1000,   // LED B0 interval in ms
    750,    // LED B1 interval in ms
    333,    // LED B2 interval in ms
};

// Blink interval timing for all LED's
unsigned long tBlinkTimeLast[NUM_LEDS];

// Create MCP23017 object
ErriezMCP23017 mcp = ErriezMCP23017(MCP23017_I2C_ADDRESS);


void setup()
{
    // Initialize Serial
    Serial.begin(115200);
    Serial.println(F("\nErriez MCP23017 I2C IO-expander Blink Async example"));

    // Initialize Wire
    Wire.begin();
    Wire.setClock(400000);

    // Initialize MCP23017
    while (!mcp.begin()) {
        Serial.print(F("Error: MCP23017 not found, code: "));
        Serial.println(mcp.getI2CStatus());
        delay(3000);
    }

    // LED pins output
    for (uint8_t pin = 0; pin < NUM_LEDS; pin++) {
        mcp.pinMode(LED0_PIN + pin, OUTPUT);
    }
}

void loop()
{
    unsigned long tNow;

    // Read current time in milliseconds
    tNow = millis();

    // Set LED output state asynchronous without blocking the application loop
    for (uint8_t led = 0; led < NUM_LEDS; led++) {
        // Toggle LED on fixed interval
        if ((tNow - tBlinkTimeLast[led]) > ledBlinkInterval[led]) {
            // Store current time LED changed
            tBlinkTimeLast[led] = tNow;

            // Toggle LED
            mcp.pinToggle(LED0_PIN + led);
        }
    }

    // Application can do other things here
}
