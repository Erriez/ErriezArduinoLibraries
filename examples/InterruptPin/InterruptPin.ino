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
 * \brief External interrupt example for Arduino
 * \details 
 *    Source: https://github.com/Erriez/ErriezArduinoLibrariesAndSketches
 *    Doc:    https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 */

// Uno, Nano, Mini, other 328-based: pin D2 (INT0) or D3 (INT1)
// Leonardo: pin D7 (INT4)
// ESP8266 / NodeMCU / WeMos D1&R2: pin D3 (GPIO0)
#if defined(__AVR_ATmega328P__)
#define INT_PIN     2
#elif defined(ARDUINO_AVR_LEONARDO)
#define INT_PIN     7
#else
#define INT_PIN     0 // GPIO0 pin for ESP8266 targets
#endif

static bool ledPinState = LOW;


void PinInterruptHandler()
{
    ledPinState ^= HIGH;
    digitalWrite(LED_BUILTIN, ledPinState);
}

void setup() 
{
    pinMode(LED_BUILTIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), PinInterruptHandler, FALLING);
}

void loop()
{
  
}
