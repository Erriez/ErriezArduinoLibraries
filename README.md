# Erriez Arduino libraries and sketches

This is Erriez Arduino libraries and sketches repository.

All libraries are built and tested from scratch without dependencies on 
third-party libraries.

The libraries and sketches can be build with CLion by using CMake as well as
the Arduino IDE.

## Sketches

### Blink
[Blink sketch](https://github.com/Erriez/ArduinoLibrariesAndSketches/tree/master/Blink)

### PIR (Passive Infrared Sensor) movement sketch
[PIR  movement sensor sketch](https://github.com/Erriez/ArduinoLibrariesAndSketches/tree/master/PIR)


## Devices

### Rotary
[3-speed rotary library](https://github.com/Erriez/ArduinoLibraryRotary)

### TM1836 LED driver
[TM1638 LED and button driver library](https://github.com/Erriez/ArduinoLibraryTM1638)

### JY-MCU JY-LKM1638
[TM1836 board with 8x 7-segment display, 8x dual color LED, 8x switches library](https://github.com/Erriez/ArduinoLibraryLKM1638)

### NRF24L01
2.4GHz wireless transceiver

[NRF24L01 API library](https://github.com/Erriez/ArduinoLibraryNRF24L01Iface)  
[NRF24L01 debug library](https://github.com/Erriez/ArduinoLibraryNRF24L01Debug)


## Diagnostics

### Printf
[printf() library](https://github.com/Erriez/ArduinoLibraryPrintf)

### Memory usage
[Memory usage library](https://github.com/Erriez/ArduinoLibraryMemoryUsage)

* Print memory usage on the serial console 
* Get stack and heap size
* Get .data and .bss sections size

### Timestamp
[Timestamp measuring library](https://github.com/Erriez/ArduinoLibraryTimestamp)


## Installation
Every Arduino library must be located in a separate GitHub repository which is
a limitation in the Arduino IDE. The easiest way to install the libraries is 
to install a Git client and clone all the projects as follows:

### Install Git client for Windows  
Download [Git client for Windows](https://git-scm.com/download/win)

### Install Git client for Linux
```bash
sudo apt-get install git
```

### Install libraries
Open a command prompt and enter the following commands:
```bash
# Windows:
cd %USERPROFILE%\Documents\Arduino\libraries
  
# Linux:
cd ~/Arduino/libraries
  
# All OSes: 
git clone https://github.com/Erriez/ArduinoLibraryLKM1638.git
git clone https://github.com/Erriez/ArduinoLibraryTM1638.git
git clone https://github.com/Erriez/ArduinoLibraryMemoryUsage.git
git clone https://github.com/Erriez/ArduinoLibraryNRF24L01Iface.git
git clone https://github.com/Erriez/ArduinoLibraryNRF24L01Debug.git
git clone https://github.com/Erriez/ArduinoLibraryRotary.git
git clone https://github.com/Erriez/ArduinoLibraryPrintf.git
git clone https://github.com/Erriez/ArduinoLibraryTimestamp.git
``` 
