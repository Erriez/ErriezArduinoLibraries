# Erriez Arduino libraries and examples

Erriez Arduino libraries and sketches contains examples and libraries for Arduino, tested on a various amount of hardware.


## Installation

Please refer to the [Wiki documentation](https://github.com/Erriez/ErriezArduinoLibraries/wiki) how to use the libraries or follow the instructions in the README.md files of the libraries.

Installation with Git from the commandline:
```
git clone https://github.com/Erriez/ErriezArduinoLibraries.git
cd ErriezArduinoLibraries
git submodule init --recursive
git submodule update --recursive

Move the libraries/* to your Arduino/libraries/ directory
```


### [Arduino Examples](https://github.com/Erriez/ErriezArduinoExamples) [![Build Status](https://travis-ci.org/Erriez/ErriezArduinoExamples.svg?branch=master)](https://travis-ci.org/Erriez/ErriezArduinoExamples)

Arduino examples and projects for several boards.


### [MH-Z19B CO2 Sensor](https://github.com/Erriez/ErriezMHZ19B) [![Build Status](https://travis-ci.org/Erriez/ErriezMHZ19B.svg?branch=master)](https://travis-ci.org/Erriez/ErriezMHZ19B)

MH-Z19B CO2 sensor library for Arduino with a small footprint. Hardware and software serial supported.

[![ErriezMHZ19B](https://raw.githubusercontent.com/Erriez/ErriezMHZ19B/master/extras/MHZ19B.png)](https://github.com/Erriez/ErriezMHZ19B)


### [Oregon THN](https://github.com/Erriez/ErriezOregonTHN128) [![Build Status](https://travis-ci.org/Erriez/ErriezOregonTHN128.svg?branch=master)](https://travis-ci.org/Erriez/ErriezOregonTHN128)

Oregon THN128 433MHz temperature sensor transmit/receive C library for Arduino.

[![ErriezOregonTHN128](https://raw.githubusercontent.com/Erriez/ErriezOregonTHN128/master/extras/OregonTHN128.png)](https://github.com/Erriez/ErriezOregonTHN128)


### [BMP280 / BME280 Sensor](https://github.com/Erriez/ErriezBMX280) [![Build Status](https://travis-ci.org/Erriez/ErriezBMX280.svg?branch=master)](https://travis-ci.org/Erriez/ErriezBMX280)

BMP280/BME280 temperature / pressure / humidity sensor library for Arduino with I2C interface.

[![ErriezBMX280](https://raw.githubusercontent.com/Erriez/ErriezBMX280/master/extras/BMX280.png)](https://github.com/Erriez/ErriezBMX280)


### [INA219 DC Power Sensor](https://github.com/Erriez/ErriezINA219) [![Build Status](https://travis-ci.org/Erriez/ErriezINA219.svg?branch=master)](https://travis-ci.org/Erriez/ErriezINA219)

INA219 I2C/SMB DC Voltage/Current/Power sensor.

[![INA219](https://raw.githubusercontent.com/Erriez/ErriezINA219/master/extras/INA219.png)](https://github.com/Erriez/ErriezINA219)


### [TTP229 Touch Keypad](https://github.com/Erriez/ErriezTTP229TouchKeypad) [![Build Status](https://travis-ci.org/Erriez/ErriezTTP229TouchKeypad.svg?branch=master)](https://travis-ci.org/Erriez/ErriezTTP229TouchKeypad)

TTP229 touch sensitive 4x4 keypad with interrupt support. Available in single 16 keys or dual 32 keys.

[![TTP229](https://raw.githubusercontent.com/Erriez/ErriezTTP229TouchKeypad/master/extras/TTP229TouchKeypad4x4.png)](https://github.com/Erriez/ErriezTTP229TouchKeypad)


### [MCP23017 16-pins I2C IO-expander](https://github.com/Erriez/ErriezMCP23017) [![Build Status](https://travis-ci.org/Erriez/ErriezMCP23017.svg?branch=master)](https://travis-ci.org/Erriez/ErriezMCP23017)

16-pin I2C IO expander with interrupt support.

[![MCP23017](https://raw.githubusercontent.com/Erriez/ErriezMCP23017/master/extras/MCP23017-pins.png)](https://github.com/Erriez/ErriezMCP23017)


### [BH1750 Light Sensor](https://github.com/Erriez/ErriezBH1750)  [![Build Status](https://travis-ci.org/Erriez/ErriezBH1750.svg?branch=master)](https://travis-ci.org/Erriez/ErriezBH1750)

I2C high-precision light sensor.

[![BH1750](https://raw.githubusercontent.com/Erriez/ErriezBH1750/master/extras/BH1750.png)](https://github.com/Erriez/ErriezBH1750)


### [HC-SR04](https://github.com/Erriez/ErriezHCSR04) [![Build Status](https://travis-ci.org/Erriez/ErriezHCSR04.svg?branch=master)](https://travis-ci.org/Erriez/ErriezHCSR04)

HC-SR04 Ultrasonic Distance sensor. Measurement range: 2..2000cm +/-1cm.

[![HC-SR04](https://raw.githubusercontent.com/Erriez/ErriezHCSR04/master/extras/HC-SR04.png)](https://github.com/Erriez/ErriezHCSR04)


### [CRC32](https://github.com/Erriez/ErriezCRC32)  [![Build Status](https://travis-ci.org/Erriez/ErriezCRC32.svg?branch=master)](https://travis-ci.org/Erriez/ErriezCRC32)

Target independent, flash and RAM size optimized CRC32 library for Arduino without CRC tables.

[![CRC32](https://raw.githubusercontent.com/Erriez/ErriezCRC32/master/extras/CRC32.png)](https://github.com/Erriez/ErriezCRC32)


### [DHT22 Temperature/Humidity Sensor](https://github.com/Erriez/ErriezDHT22)  [![Build Status](https://travis-ci.org/Erriez/ErriezDHT22.svg?branch=master)](https://travis-ci.org/Erriez/ErriezDHT22)

One-wire temperature and humidity sensor with high precision.

[![DHT22](https://raw.githubusercontent.com/Erriez/ErriezDHT22/master/extras/AM2302_DHT22_sensor.png)](https://github.com/Erriez/ErriezDHT22)


### [DS3231 high accurate I2C RTC (Real Time Clock)](https://github.com/Erriez/ErriezDS3231)  [![Build Status](https://travis-ci.org/Erriez/ErriezDS3231.svg?branch=master)](https://travis-ci.org/Erriez/ErriezDS3231)

DS3231 High accurate RTC (Real Time Clock).

[![DS3231 RTC (Real Time Clock)](https://raw.githubusercontent.com/Erriez/ErriezDS3231/master/extras/DS3231.png)](https://github.com/Erriez/ErriezDS3231)


### [DS1307 RTC (Real Time Clock)](https://github.com/Erriez/ErriezDS1307)  [![Build Status](https://travis-ci.org/Erriez/ErriezDS1307.svg?branch=master)](https://travis-ci.org/Erriez/ErriezDS1307)

DS1307 RTC (Real Time Clock).

[![DS1307 RTC (Real Time Clock)](https://raw.githubusercontent.com/Erriez/ErriezDS1307/master/extras/DS1307.png)](https://github.com/Erriez/ErriezDS1302)


### [DS1302 RTC (Real Time Clock)](https://github.com/Erriez/ErriezDS1302)  [![Build Status](https://travis-ci.org/Erriez/ErriezDS1302.svg?branch=master)](https://travis-ci.org/Erriez/ErriezDS1302)

DS1302 RTC (Real Time Clock).

[![DS1302 RTC (Real Time Clock)](https://raw.githubusercontent.com/Erriez/ErriezDS1302/master/extras/DS1302.png)](https://github.com/Erriez/ErriezDS1302)


### [LCD Keypad Shield](https://github.com/Erriez/ErriezLCDKeypadShield)  [![Build Status](https://travis-ci.org/Erriez/ErriezLCDKeypadShield.svg?branch=master)](https://travis-ci.org/Erriez/ErriezLCDKeypadShield)

2x16 character and 5 buttons LCD shield for Arduino.

[![LCDKeypadShield](https://raw.githubusercontent.com/Erriez/ErriezLCDKeypadShield/master/extras/LCDKeypadShield_board.png)](https://github.com/Erriez/ErriezLCDKeypadShield)


### [LM35 Analog Temperature Sensor](https://github.com/Erriez/ErriezLM35)  [![Build Status](https://travis-ci.org/Erriez/ErriezLM35.svg?branch=master)](https://travis-ci.org/Erriez/ErriezLM35)

LM35 analog temperature sensor library for Arduino.

[![LM35](https://raw.githubusercontent.com/Erriez/ErriezLM35/master/extras/LM35_pins.png)](https://github.com/Erriez/ErriezLM35)


### [Memory Usage](https://github.com/Erriez/ErriezMemoryUsage)  [![Build Status](https://travis-ci.org/Erriez/ErriezMemoryUsage.svg?branch=master)](https://travis-ci.org/Erriez/ErriezMemoryUsage)

Memory usage diagnostics library for Arduino.

[![Memory Usage](https://raw.githubusercontent.com/Erriez/ErriezMemoryUsage/master/extras/ErriezMemoryUsage.png)](https://github.com/Erriez/ErriezMemoryUsage)


### [RobotDyn 4-digit display](https://github.com/Erriez/ErriezRobotDyn4DigitDisplay)  [![Build Status](https://travis-ci.org/Erriez/ErriezRobotDyn4DigitDisplay.svg?branch=master)](https://travis-ci.org/Erriez/ErriezRobotDyn4DigitDisplay)

RobotDyn 4-digit display library for Arduino with a two-wire TM1637 LED controller.

[![RobotDyn 4-digit display](https://raw.githubusercontent.com/Erriez/ErriezRobotDyn4DigitDisplay/master/extras/ErriezRobotDyn4DigitDisplay.png)](https://github.com/Erriez/ErriezRobotDyn4DigitDisplay)


### [RobotDyn Keypad 3x4 with analog output](https://github.com/Erriez/ErriezRobotDynKeypad3x4Analog)  [![Build Status](https://travis-ci.org/Erriez/ErriezRobotDynKeypad3x4Analog.svg?branch=master)](https://travis-ci.org/ErriezRobotDynKeypad3x4Analog)

RobotDyn Keypad 3x4 with analog output library for Arduino.

[![RobotDyn Keypad 3x4 Analog](https://raw.githubusercontent.com/Erriez/ErriezRobotDynKeypad3x4Analog/master/extras/RobotDynKeypad3x4Analog.png)](https://github.com/Erriez/ErriezRobotDynKeypad3x4Analog)


### [Rotary Encoder full step](https://github.com/Erriez/ErriezRotaryEncoderFullStep)  [![Build Status](https://travis-ci.org/Erriez/ErriezRotaryEncoderFullStep.svg?branch=master)](https://travis-ci.org/Erriez/ErriezRotaryEncoderFullStep)

3 speed full step Rotary Encoder with button.

[![Rotary Encoder full step](https://raw.githubusercontent.com/Erriez/ErriezRotaryEncoderFullStep/master/extras/RotaryEncoder.png)](https://github.com/Erriez/ErriezRotaryEncoderFullStep)


### [Rotary Encoder half step](https://github.com/Erriez/ErriezRotaryEncoderHalfStep)  [![Build Status](https://travis-ci.org/Erriez/ErriezRotaryEncoderHalfStep.svg?branch=master)](https://travis-ci.org/Erriez/ErriezRotaryEncoderHalfStep)

3 speed half step Rotary Encoder.

[![Rotary Encoder half step](https://raw.githubusercontent.com/Erriez/ErriezRotaryEncoderHalfStep/master/extras/RotaryEncoder.png)](https://github.com/Erriez/ErriezRotaryEncoderHalfStep)


### [Serial Terminal](https://github.com/Erriez/ErriezSerialTerminal)  [![Build Status](https://travis-ci.org/Erriez/ErriezSerialTerminal.svg?branch=master)](https://travis-ci.org/Erriez/ErriezSerialTerminal)

A universal Serial Terminal library for Arduino to parse ASCII commands and arguments.

[![Serial Terminal](https://raw.githubusercontent.com/Erriez/ErriezSerialTerminal/master/extras/ScreenshotSerialTerminal.png)](https://github.com/Erriez/ErriezSerialTerminal)


### [Timestamp](https://github.com/Erriez/ErriezTimestamp)  [![Build Status](https://travis-ci.org/Erriez/ErriezTimestamp.svg?branch=master)](https://travis-ci.org/Erriez/ErriezTimestamp)

Timestamp library to measure execution duration, useful for benchmarks.

[![Timestamp](https://raw.githubusercontent.com/Erriez/ErriezTimestamp/master/extras/timestamp.png)](https://github.com/Erriez/ErriezTimestamp)


### [TM1637 Key/LED driver](https://github.com/Erriez/ErriezTM1637)  [![Build Status](https://travis-ci.org/Erriez/ErriezTM1637.svg?branch=master)](https://travis-ci.org/Erriez/ErriezTM1637)

2-wire 6x8 LED's and 8x2 key-scan controller.

[![TM1637](https://raw.githubusercontent.com/Erriez/ErriezTM1637/master/extras/TM1637_pins.jpg)](https://github.com/Erriez/ErriezTM1637)


### [TM1638 Key/LED driver](https://github.com/Erriez/ErriezTM1638)  [![Build Status](https://travis-ci.org/Erriez/ErriezTM1638.svg?branch=master)](https://travis-ci.org/Erriez/ErriezTM1638)

3-wire 10x8 LED's and 8x3 key-scan controller.

[![TM1638](https://raw.githubusercontent.com/Erriez/ErriezTM1638/master/extras/TM1638_pins.jpg)](https://github.com/Erriez/ErriezTM1638)


### [JY-MCU JY-LKM1638 board](https://github.com/Erriez/ErriezLKM1638)  [![Build Status](https://travis-ci.org/Erriez/ErriezLKM1638.svg?branch=master)](https://travis-ci.org/Erriez/ErriezLKM1638)

8 digits, 8 dual color LED's and 8 switches JY-MCU JY-LKM1638 board with TM1638 chip library for Arduino.

[![JY-MCU JY-LKM1638 board](https://raw.githubusercontent.com/Erriez/ErriezLKM1638/master/extras/LKM1638_board.jpg)](https://github.com/Erriez/ErriezLKM1638)


### [printf for AVR](https://github.com/Erriez/ErriezPrintf)  [![Build Status](https://travis-ci.org/Erriez/ErriezPrintf.svg?branch=master)](https://travis-ci.org/Erriez/ErriezPrintf)

printf() library for Arduino AVR targets.
