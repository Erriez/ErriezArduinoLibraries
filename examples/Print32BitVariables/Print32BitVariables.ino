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
 * \brief Print 32-bit variables
 * \details
 *      Source: https://github.com/Erriez/ArduinoLibrariesAndSketches
 */

// Function prototypes
void printHex32(uint32_t val);
void printDec32(uint32_t val);


void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ;
    }
    Serial.println(F("Print 32-bit variables example"));

    // Print some 32-bit values
    printDec32(12345678UL);
    printHex32(0x12345678UL);
}

void loop()
{
    // Empty
}

void printHex32(uint32_t val)
{
    Serial.print("0x");
    for (int8_t i = 3; i >= 0; i--) {
        uint8_t c = (uint8_t)(val >> (i * 8));
        if (c < 0x10) {
            Serial.print("0");
        }
        Serial.print(c, HEX);
    }
    Serial.println();
}

void printDec32(uint32_t val)
{
    char buf[9];

    snprintf_P(buf, sizeof(buf), PSTR("%lu"), val);
    Serial.println(buf);
}
