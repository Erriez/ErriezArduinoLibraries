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
 * \brief DS1820 one-wire temperature sensor
 * \details
 *      Source: https://github.com/Erriez/ArduinoLibrariesAndSketches
 */

#include <OneWire.h>
#include <DallasTemperature.h>

// One wire pin
#define ONE_WIRE_BUS    D7

// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature owSensors(&oneWire);

// DS1820 device address
DeviceAddress ds1820DeviceAddress;

// Temperature resolution
#define DS1820_RESOLUTION   12


void setup(void) 
{ 
    // Initialize serial port 
    Serial.begin(115200); 
    Serial.println("DS1820 temperature sensor example");
    
    owSensors.begin();
    
    owSensors.getAddress(ds1820DeviceAddress, 0);
    owSensors.setResolution(ds1820DeviceAddress, DS1820_RESOLUTION);

    owSensors.setWaitForConversion(false);
}

void loop(void) 
{ 
    owSensors.requestTemperatures();

    // Non-blocking
    delay(1000); 

    Serial.print("DS1820 temperature: "); 
    Serial.println(owSensors.getTempCByIndex(0));

    delay(1000); 
} 
