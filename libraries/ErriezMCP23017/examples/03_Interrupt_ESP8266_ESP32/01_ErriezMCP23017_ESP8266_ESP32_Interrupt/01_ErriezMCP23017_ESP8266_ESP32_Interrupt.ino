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
  *      This example demonstrates MCP23017 interrupts for the ESP8266 or ESP32:
  *      - Pin change
  *      - Pin edge falling
  *      - Pin edge rising
  *      - Low-power sleep mode
  *      - MCP23017 pin INTA connected to any pin with interrupt support
  *
  *      Notes:
  *      - MCP23017 pin INTB is not supported in this library.
  *      - MCP23017 does NOT support interrupt edge.
  *      - MCP23017 supports level interrupt, but not supported by this library.
  *      - MCP23017 hardware limitations are handled in the library by software.
  *
  *      Hardware:
  *      - All 16-pins input with pin interrupt enabled.
  *      - MCP23017 INTA pin connected to any pin with interrupt support.
  */

#include <Arduino.h>
#include <Wire.h>
#include <ErriezMCP23017.h>

#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
// ESP8266 / ESP32: All interrupt pins
#define INT_PIN                 2 // GPIO0 (NodeMCU: D4)
#else
#error "This example has been tested on ESP8266 and ESP32 targets only"
#endif

// PORTA: pins 0..7
// PORTB: pins 8..15
#define INTERRUPT_PINS          0x00FF  // A0..A7
#define LED_PINS                0xFF00  // B0..B7

// Default I2C Address [default = 0x20]
#define MCP23017_I2C_ADDRESS    0x20

// Polarity interrupt INTA pin FALLING [default] or RISING
#define INTA_POLARITY           FALLING

// Reset MCP23017 on startup [default = true]
#define MCP23017_RESET_STARTUP  true

// Enable on-chip low power (Comment to disable)
#define ENABLE_LOW_POWER

// Interrupt flag shared with interrupt handler must be volatile
volatile bool interruptFlag = false;

// Create MCP23017 object
ErriezMCP23017 mcp = ErriezMCP23017(MCP23017_I2C_ADDRESS);


static ICACHE_RAM_ATTR void mcpInterruptHandler()
{
    // Keep interrupt handling short by setting a flag. Interrupt must be
    // handled in main loop.
    interruptFlag = true;
}

void setup()
{
    // Initialize Serial
    Serial.begin(115200);
    Serial.println(F("\nErriez MCP23017 I2C IO-expander Interrupt Edge"));

    // Initialize Wire
    Wire.begin();
    Wire.setClock(400000);

    // Initialize MCP23017
    while (!mcp.begin(MCP23017_RESET_STARTUP)) {
        Serial.print(F("Error: MCP23017 not found, code: "));
        Serial.println(mcp.getI2CStatus());
        delay(3000);
    }

    // Set polarity MCP23017 INTA pin:
    //   false: INTA active low when a pin interrupt occurs
    //   true:  INTA active high when a pin interrupt occurs
    if (INTA_POLARITY == RISING) {
        mcp.setInterruptPolarityINTA(true);
    }

    // LED pins output
    mcp.setPortDirection(LED_PINS);

    // Enable pins pull-up
    mcp.setPortPullup(INTERRUPT_PINS);

    // Enable input interrupt on pins
    mcp.setPortInterruptEnable(INTERRUPT_PINS);

    // Enable MCU interrupt pin connected to MCP23017 INTA pin
    attachInterrupt(digitalPinToInterrupt(INT_PIN),
                    mcpInterruptHandler, INTA_POLARITY);

    // Dump registers
    mcp.dumpRegisters(&Serial);

    // End setup
    Serial.println(F("Waiting for INTA interrupt..."));
}

void loop()
{
    char line[20];

    if (interruptFlag) {
        // Clear interrupt flag
        interruptFlag = false;

        // Print state/changed/falling/rising when at least one pin changed
        if (mcp.interruptINTA()) {
            // Print pin states
            snprintf_P(line, sizeof(line), PSTR("State:   0x%04x"), mcp.portState);
            Serial.println(line);

            snprintf_P(line, sizeof(line), PSTR("Changed: 0x%04x"), mcp.pinsChanged);
            Serial.println(line);

            snprintf_P(line, sizeof(line), PSTR("Falling: 0x%04x"), mcp.pinsFalling);
            Serial.println(line);

            snprintf_P(line, sizeof(line), PSTR("Rising:  0x%04x\n"), mcp.pinsRising);
            Serial.println(line);

            // Toggle LED's on falling edge input pins (no debouncing)
            if (mcp.pinsFalling) {
                mcp.portToggle(mcp.pinsFalling << 8);
            }
        }
    }

    // Simulate application processing time. The longer the duration of not
    // servicing the MCP23017 interrupt, the higher the chance of lossing
    // interrupt pin changes.
    // delay(50);
}
