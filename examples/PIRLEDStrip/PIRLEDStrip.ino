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
 * \brief LED strip with PIR example for Arduino
 * \details
 *    Required library: https://github.com/Erriez/ErriezArduinoLibrariesAndSketches
 *    Required hardware: Arduino Pro or Pro Mini 3.3V 8MHz
 */


#include <LowPower.h>

// Pin defines
#define INT_PIN     2
#define PWM_PIN     5

volatile bool pirInterrupt = false;
volatile int pirInterruptTicks = 0;
int ledPWMTarget = 0;
int ledPWMCurrent = 0;

#define SLEEP_TIME    75


void PinInterruptHandler()
{
    pirInterrupt = true;
}

void setup() 
{
    // Serial.begin(115200);
    // Serial.println(F("PIR LED strip"));
    
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(A0, INPUT_PULLUP);
    pinMode(INT_PIN, INPUT_PULLUP);
    pinMode(PWM_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(INT_PIN), PinInterruptHandler, RISING);
}

void loop()
{   
    uint16_t vcc;
    uint16_t light;
    
    digitalWrite(LED_BUILTIN, digitalRead(INT_PIN));

    if (pirInterrupt) {
        vcc = readVcc();
        // Serial.println(vcc, DEC);

        light = analogRead(A0);
        // Serial.println(light, DEC);

        if (light < 600) {
            //Serial.println(F("TOO MUCH LIGHT"));
            ledPWMTarget = 0;
            pirInterruptTicks = 10 / SLEEP_TIME;
        } else if (vcc < 3000) {
            //Serial.println(F("LOW BATTERY"));
            ledPWMTarget = 5;
            pirInterruptTicks = 10 / SLEEP_TIME;
        } else {
            ledPWMTarget = 200;
            pirInterruptTicks = 20000 / SLEEP_TIME;
        }

        pirInterrupt = false;
    }

    if (pirInterruptTicks) {
        pirInterruptTicks--;
    }

    if ((pirInterruptTicks == 0) && (ledPWMTarget)) {
        ledPWMTarget = 0;
        pirInterruptTicks = 15000 / SLEEP_TIME;
    }

    if (ledPWMTarget > ledPWMCurrent) {
        if ((ledPWMTarget - ledPWMCurrent) > 40) {
            ledPWMCurrent += 3;
        } else {
            ledPWMCurrent++;
        }
    } else if (ledPWMTarget < ledPWMCurrent) {
        if ((ledPWMCurrent - ledPWMTarget) > 75) {
            ledPWMCurrent -= 3;
        } else if ((ledPWMCurrent - ledPWMTarget) > 50) {
            ledPWMCurrent -= 2;
        } else {
            ledPWMCurrent--;
            if (ledPWMCurrent == 15) {
                delay(6000);
            }
        }
    }

    analogWrite(PWM_PIN, ledPWMCurrent);

    // Serial.println(pirInterruptTicks);
    // Serial.println(ledPWMTarget);

    if (ledPWMTarget == 0) {
        digitalWrite(LED_BUILTIN, LOW);
    }

    if (ledPWMCurrent) {
        delay(SLEEP_TIME);
    } else {
        // Serial.flush();
        // LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
    }
}

uint16_t readVcc() 
{
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
      ADMUX = _BV(MUX5) | _BV(MUX0);
    #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
      ADMUX = _BV(MUX3) | _BV(MUX2);
    #else
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif  
  
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC)) {
        ;
    }
  
    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
    uint8_t high = ADCH; // unlocks both
  
    uint16_t result = (high << 8) | low;
  
    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return result; // Vcc in millivolts
}

