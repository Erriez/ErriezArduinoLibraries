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
 * \brief 4x20 character LCD with PCF8574 I2C IO expander for Arduino
 * \details
 *      Source:   https://github.com/Erriez/ErriezArduinoLibrariesAndSketches
 *      Library:  https://github.com/fmalpartida/New-LiquidCrystal
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h> // https://github.com/fmalpartida/New-LiquidCrystal.git
#include <LCD.h>

// Create LCD object with I2C PCF8574 IO-expander
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Arduino UNO pins:
//    SDA: pin A4
//    SCL: pin A5

void setup()
{
    Wire.begin();
    Wire.setClock(100000);

    lcd.begin(20, 4);
    lcd.backlight();
    lcd.clear();
    lcd.home();
    lcd.print("Hello world");
    lcd.setCursor(3, 4);
    lcd.print("1234");
}

void loop()
{

}
