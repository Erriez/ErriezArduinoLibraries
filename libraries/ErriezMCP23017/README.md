# MCP23017 16-pin I2C IO-expander library for Arduino

[![Build Status](https://travis-ci.org/Erriez/ErriezMCP23017.svg?branch=master)](https://travis-ci.org/Erriez/ErriezMCP23017)

This is a MCP23017 16-pin I2C IO-expander library for Arduino with interrupt change/edge support and extensive examples.

![MCP23017 16-pin I2C IO-expander](https://raw.githubusercontent.com/Erriez/ErriezMCP23017/master/extras/MCP23017-pins.png)


## Library features

* I2C interface
* Input/output/read/write/toggle/mask control per pin or per 16-pins
* Configurable pullup per pin
* Interrupt change/falling/rising per pin
* Interrupt edge handled by the library, because the chip supports pin change / level only.
* Low-power support
* Generic examples / AVR / ESP8266 / ESP32 support


## Hardware

The following targets are supported:

* AVR: UNO, MINI, Pro Mini 8/16 MHz, ATMega2560, Leonardo
* ARM: DUE
* ESP8266: Mini D1 & D2, NodeMCU
* ESP32: Lolin D32


## Examples

Extensive examples are located [here](https://github.com/Erriez/ErriezMCP23017/tree/master/examples).


## Documentation

- [Online HTML](https://erriez.github.io/ErriezMCP23017)
- [Download PDF](https://github.com/Erriez/ErriezMCP23017/raw/master/ErriezMCP23017.pdf)


**Getting started LED blink**

```c++
#include <Arduino.h>
#include <Wire.h>
#include <ErriezMCP23017.h>

// PORTA: pins 0..7
// PORTB: pins 8..15
#define LED_PIN                 8 // Pin B0

// Default I2C Address 0x20
#define MCP23017_I2C_ADDRESS    0x20

// Create MCP23017 object
ErriezMCP23017 mcp = ErriezMCP23017(MCP23017_I2C_ADDRESS);

void setup()
{
    // Initialize Wire
    Wire.begin();
    Wire.setClock(400000);

    // Initialize MCP23017
    while (!mcp.begin()) {
        // MCP23017 not detected
        delay(3000);
    }

    // LED pin output
    mcp.pinMode(LED_PIN, OUTPUT);
}

void loop()
{
    mcp.digitalWrite(LED_PIN, HIGH);
    delay(1000);
    mcp.digitalWrite(LED_PIN, LOW);
    delay(1000);
}
```


## Library installation

Please refer to the [Wiki](https://github.com/Erriez/ErriezArduinoLibrariesAndSketches/wiki) page.


## Other Arduino Libraries and Sketches from Erriez

[Erriez Libraries and Sketches](https://github.com/Erriez/ErriezArduinoLibrariesAndSketches)

