# Erriez Arduino libraries and sketches

Erriez Arduino libraries and sketches contains examples and optimized libraries for Arduino, tested on a various amount of hardware.

Please refer to the [Wiki documentation](https://github.com/Erriez/ErriezArduinoLibrariesAndSketches/wiki) how to use the libraries or follow the instructions in the README.md files of the libraries.


### Install Erriez Libraries using GIT

1. Start the Arduino IDE and click ```Sketch | Preferences```:
2. Copy the path in the ```Sketchbook location``` edit box, for example ```C:\Users\User\Documents\Arduino``` or ```/home/user/Arduino```.
3. Open a command prompt and type the following commands:

```
# Go to the copied Sketchbook location directory and replace <Sketchbook location> with the copied path:
# cd <Sketchbook location>
# For example on Windows:
cd C:\Users\%USERNAME%\Documents\Arduino
# For example on Linux:
cd ~/Arduino

git clone https://github.com/Erriez/ErriezArduinoLibrariesAndSketches.git

# Clone projects into libraries directory
# For example on Windows: C:\Users\%USERNAME%\Documents\Arduino\libraries
# For example on Linux: ~/Arduino/libraries
cd libraries
git clone https://github.com/Erriez/ErriezBH1750.git
git clone https://github.com/Erriez/ErriezDHT22.git
git clone https://github.com/Erriez/ErriezDS3231.git
git clone https://github.com/Erriez/ErriezLCDKeypadShield.git
git clone https://github.com/Erriez/ErriezLKM1638.git
git clone https://github.com/Erriez/ErriezLM35.git
git clone https://github.com/Erriez/ErriezRobotDynKeypad3x4Analog.git
git clone https://github.com/Erriez/ErriezRotaryEncoderFullStep.git
git clone https://github.com/Erriez/ErriezRotaryEncoderHalfStep.git
git clone https://github.com/Erriez/ErriezTimestamp.git
git clone https://github.com/Erriez/ErriezTM1637.git
git clone https://github.com/Erriez/ErriezTM1638.git
```

3. Restart the Arduino IDE.


### [BH1750](https://github.com/Erriez/ErriezBH1750)  [![Build Status](https://travis-ci.org/Erriez/ErriezBH1750.svg?branch=master)](https://travis-ci.org/Erriez/ErriezBH1750)

I2C high-precision light sensor.

[![BH1750](https://raw.githubusercontent.com/Erriez/ErriezBH1750/master/extras/BH1750.png)](https://github.com/Erriez/ErriezBH1750)


### [DHT22](https://github.com/Erriez/ErriezDHT22)  [![Build Status](https://travis-ci.org/Erriez/ErriezDHT22.svg?branch=master)](https://travis-ci.org/Erriez/ErriezDHT22)

One-wire temperature and humidity sensor with high precision.

[![DHT22](https://raw.githubusercontent.com/Erriez/ErriezDHT22/master/extras/AM2302_DHT22_sensor.png)](https://github.com/Erriez/ErriezDHT22)


### [DS1302 RTC (Real Time Clock)](https://github.com/Erriez/ErriezDS1302)  [![Build Status](https://travis-ci.org/Erriez/ErriezDS1302.svg?branch=master)](https://travis-ci.org/Erriez/ErriezDS1302)

DS1302 inaccurate RTC (Real Time Clock).

[![DS1302 RTC (Real Time Clock)](https://raw.githubusercontent.com/Erriez/ErriezD1302/master/extras/DS1302.png)](https://github.com/Erriez/ErriezDS1302)


### [DS3231 high accurate I2C RTC (Real Time Clock)](https://github.com/Erriez/ErriezDS3231)  [![Build Status](https://travis-ci.org/Erriez/ErriezDS3231.svg?branch=master)](https://travis-ci.org/Erriez/ErriezDS3231)

DS3231 High accurate RTC (Real Time Clock).

[![DS3231 RTC (Real Time Clock)](https://raw.githubusercontent.com/Erriez/ErriezDS3231/master/extras/DS3231.png)](https://github.com/Erriez/ErriezDS3231)


### [LCD Keypad Shield](https://github.com/Erriez/ErriezLCDKeypadShield)  [![Build Status](https://travis-ci.org/Erriez/ErriezLCDKeypadShield.svg?branch=master)](https://travis-ci.org/Erriez/ErriezLCDKeypadShield)

2x16 character and 5 buttons LCD shield for Arduino.

[![LCDKeypadShield](https://raw.githubusercontent.com/Erriez/ErriezLCDKeypadShield/master/extras/LCDKeypadShield_board.png)](https://github.com/Erriez/ErriezLCDKeypadShield)


### [LM35](https://github.com/Erriez/ErriezLM35)  [![Build Status](https://travis-ci.org/Erriez/ErriezLM35.svg?branch=master)](https://travis-ci.org/Erriez/ErriezLM35)

LM35 analog temperature sensor library for Arduino.

[![LM35](https://raw.githubusercontent.com/Erriez/ErriezLM35/master/extras/LM35_pins.png)](https://github.com/Erriez/ErriezLM35)


### [RobotDyn Keypad 3x4 with analog output](https://github.com/Erriez/ErriezRotaryEncoderFullStep)  [![Build Status](https://travis-ci.org/Erriez/ErriezRobotDynKeypad3x4Analog.svg?branch=master)](https://travis-ci.org/ErriezRobotDynKeypad3x4Analog)

RobotDyn Keypad 3x4 with analog output library for Arduino.

[![RobotDyn Keypad 3x4 Analog](https://raw.githubusercontent.com/Erriez/ErriezRobotDynKeypad3x4Analog/master/extras/RobotDynKeypad3x4Analog.png)](https://github.com/Erriez/ErriezRotaryEncoderFullStep)


### [Rotary Encoder full step](https://github.com/Erriez/ErriezRotaryEncoderFullStep)  [![Build Status](https://travis-ci.org/Erriez/ErriezRotaryEncoderFullStep.svg?branch=master)](https://travis-ci.org/Erriez/ErriezRotaryEncoderFullStep)

3 speed full step Rotary Encoder with button.

[![Rotary Encoder full step](https://raw.githubusercontent.com/Erriez/ErriezRotaryEncoderFullStep/master/extras/RotaryEncoder.png)](https://github.com/Erriez/ErriezRotaryEncoderFullStep)


### [Rotary Encoder half step](https://github.com/Erriez/ErriezRotaryEncoderHalfStep)  [![Build Status](https://travis-ci.org/Erriez/ErriezRotaryEncoderHalfStep.svg?branch=master)](https://travis-ci.org/Erriez/ErriezRotaryEncoderHalfStep)

3 speed half step Rotary Encoder.

[![Rotary Encoder half step](https://raw.githubusercontent.com/Erriez/ErriezRotaryEncoderHalfStep/master/extras/RotaryEncoder.png)](https://github.com/Erriez/ErriezRotaryEncoderHalfStep)


### [Serial Terminal](https://github.com/Erriez/ErriezSerialTerminal)  [![Build Status](https://travis-ci.org/Erriez/ErriezSerialTerminal.svg?branch=master)](https://travis-ci.org/Erriez/ErriezSerialTerminal)

A universal Serial Terminal library for Arduino to parse ASCII commands and arguments.

[![Serial Terminal](https://raw.githubusercontent.com/Erriez/ErriezSerialTerminal/master/extras/ScreenshotSerialTerminal.png)](https://github.com/Erriez/ErriezSerial)


### [Timestamp](https://github.com/Erriez/ErriezTimestamp)  [![Build Status](https://travis-ci.org/Erriez/ErriezTimestamp.svg?branch=master)](https://travis-ci.org/Erriez/ErriezTimestamp)

Timestamp library to measure execution duration, useful for benchmarks.

[![Timestamp](https://raw.githubusercontent.com/Erriez/ErriezTimestamp/master/extras/timestamp.png)](https://github.com/Erriez/ErriezTimestamp)


### [TM1637](https://github.com/Erriez/ErriezTM1637)  [![Build Status](https://travis-ci.org/Erriez/ErriezTM1637.svg?branch=master)](https://travis-ci.org/Erriez/ErriezTM1637)

2-wire 6x8 LED's and 8x2 key-scan controller.

[![TM1637](https://raw.githubusercontent.com/Erriez/ErriezTM1637/master/extras/TM1637_pins.jpg)](https://github.com/Erriez/ErriezTM1637)


### [TM1638](https://github.com/Erriez/ErriezTM1638)  [![Build Status](https://travis-ci.org/Erriez/ErriezTM1638.svg?branch=master)](https://travis-ci.org/Erriez/ErriezTM1638)

3-wire 10x8 LED's and 8x3 key-scan controller.

[![TM1638](https://raw.githubusercontent.com/Erriez/ErriezTM1638/master/extras/TM1638_pins.jpg)](https://github.com/Erriez/ErriezTM1638)


### [JY-MCU JY-LKM1638 board](https://github.com/Erriez/ErriezLKM1638)  [![Build Status](https://travis-ci.org/Erriez/ErriezLKM1638.svg?branch=master)](https://travis-ci.org/Erriez/ErriezLKM1638)

8 digits, 8 dual color LED's and 8 switches JY-MCU JY-LKM1638 board with TM1638 chip library for Arduino.

[![JY-MCU JY-LKM1638 board](https://raw.githubusercontent.com/Erriez/ErriezLKM1638/master/extras/LKM1638_board.jpg)](https://github.com/Erriez/ErriezLKM1638)

