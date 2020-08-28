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
 * \brief DS1307 I2C RTC example
 * \details
 *      Source: https://github.com/Erriez/ArduinoLibrariesAndSketches
 */

#include <Wire.h>

#define DS1307_ADDRESS  (0xD0 >> 1)

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ;
    }
    pinMode(LED_BUILTIN, OUTPUT);

    DS1307_WriteRegister(0x00, 0x00);

    /*while (1) {
      Serial.println(DS1307_ReadRegister(0), HEX);
      delay(1000);
    }*/

    while (1) {
        uint8_t buf[8];

        DS1307_ReadBuffer(0, buf, sizeof(buf));

        for (uint8_t i = 0; i < sizeof(buf); i++) {
            Serial.println(buf[i], HEX);
        }
        Serial.println();

        delay(1000);
    }
}

static void DS1307_ReadBuffer(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.begin();
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.requestFrom(DS1307_ADDRESS, len);
    while (Wire.available()) {
        *buf++ = Wire.read();
    }
    Wire.endTransmission(true);
}

static uint8_t DS1307_ReadRegister(uint8_t reg)
{
    uint8_t value;

    Wire.begin();
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.requestFrom(DS1307_ADDRESS, 1);
    value = Wire.read();
    Wire.endTransmission(true);

    return value;
}

static void DS1307_WriteRegister(uint8_t reg, uint8_t value)
{
    Wire.begin();
    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}
