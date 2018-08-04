/*
 * MIT License
 *
 * Copyright (c) 2018 Erriez
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
 * \brief Assertion example
 * \details
 *      This is an assertion example prints the filename and line number of the error. Output:
 *
 *          Assertion example
 *          1...Success!
 *          2...Success!
 *          3...Success!
 *          4...Success!
 *          5...Success!
 *          6...Success!
 *          7...Success!
 *          8...Success!
 *          9...Success!
 *          10...Failure: Assert.ino:78
 *
 *      Source: https://github.com/Erriez/ArduinoLibrariesAndSketches
 */

static int count = 1;

#define Success     false
#define Failure      true

#define CHK(err) {                      \
    if (err == Success) {               \
        while (0)                       \
            ;                           \
    } else {                            \
        Serial.print(F("Failure: "));     \
        Serial.print((strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__));     \
        Serial.print(F(":"));           \
        Serial.println(__LINE__);       \
        Serial.flush();                 \
        cli();                          \
        while (1)                       \
            ;                           \
    }                                   \
}

bool foo()
{
    if (count++ == 10) {
        // Return a failure
        return Failure;
    }

    // Return success
    return Success;
}

void setup()
{
    // Initialize serial port
    Serial.begin(115200);
    while (!Serial) {
        ;
    }
    Serial.println(F("Assertion example"));
}

void loop()
{
    Serial.print(count);
    Serial.print(F("..."));
    CHK(foo());
    Serial.println(F("Success!"));
}
