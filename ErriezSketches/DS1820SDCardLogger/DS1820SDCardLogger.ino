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
 * \brief DS1820 one-wire temperature SD-card logger
 * \details
 *      Source: https://github.com/Erriez/ArduinoLibrariesAndSketches
 */

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <avr/power.h>
#include <LowPower.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ErriezDS3231.h>

#define DS3231_INT_PIN      2
#define ONE_WIRE_BUS        8
#define LED_PIN             9

// SD-card pin
#define SD_CARD_CS_PIN      10
#define SD_CARD_POWER_PIN   4

DS3231 rtc;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds1820(&oneWire);

// Create and initialize date time object
static DateTime_t dt = {
    .second = 0,
    .minute = 55,
    .hour = 21,
    .dayWeek = 2,  // 1 = Monday
    .dayMonth = 5,
    .month = 1,
    .year = 2019
};

//#define RTC_SET_DATE_TIME

// Alarm interrupt flag must be volatile
static volatile bool rtcInterrupt = true;

static byte keep_SPCR;

static double temperature = 0.0;
static double temperatureMin = 100.0;
static double temperatureMax = -50.0;

static void ledInit()
{
    pinMode(LED_PIN, OUTPUT);
    ledOff();
}

static void ledOn()
{
    digitalWrite(LED_PIN, LOW);
}

static void ledOff()
{
    digitalWrite(LED_PIN, HIGH);
}

static void ledBlink(uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        ledOn();
        delay(100);
        ledOff();
        delay(100);
    }
}

static void rtcHandler()
{
    // Set global interrupt flag
    rtcInterrupt = true;
}

static void readTemperature()
{
    for (uint8_t i = 0; i < 10; i++) {
        ds1820.requestTemperatures();

        // Wait at least 750ms before reading temperature
        Serial.flush();
        LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_ON);
        LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_ON);
        LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);

        temperature = ds1820.getTempCByIndex(0);
        if ((temperature >= -50.0) && (temperature <= 80.0)) {
            break;
        }
    }

    if ((temperature < -50.0) || (temperature >= 80.0)) {
        ledBlink(3);
        return;
    }

    // Calculate minimum
    if (temperature < temperatureMin) {
        temperatureMin = temperature;
    }

    // Calculate maximum
    if (temperature > temperatureMax) {
        temperatureMax = temperature;
    }
}

static void readRTCDateTime()
{
    // Read RTC date and time from RTC
    if (!rtc.getDateTime(&dt)) {
        Serial.println(F("Error: RTC read date time failed"));
        ledBlink(4);
    }
}

static void printDateTimeTemperature()
{
    char line[32];

    snprintf(line, sizeof(line),
             "%d-%02d-%02d, %d:%02d:%02d, ", 
             dt.year, dt.month, dt.dayMonth, 
             dt.hour, dt.minute, dt.second);
    Serial.print(line);
    Serial.print(temperature);
    Serial.println(F("C"));
}

void turnOnSDcard()
{
    // https://thecavepearlproject.org/2017/05/21/switching-off-sd-cards-for-low-power-data-logging

    // Turn SD-card power on via 1K resistor to basis NPN transistor
    // Collector connected to GND SD-card.
    pinMode(SD_CARD_POWER_PIN, OUTPUT);
    digitalWrite(SD_CARD_POWER_PIN, HIGH);
    delay(6);

    // Some cards will fail on power-up unless SS is pulled up  ( &  D0/MISO as well? )
    DDRB |= (1<<DDB5) | (1<<DDB3) | (1<<DDB2); // set SCLK(D13), MOSI(D11) & SS(D10) as OUTPUT
    // Note: | is an OR operation so  the other pins stay as they were.                (MISO stays as INPUT)
    PORTB &= ~(1<<DDB5);  // disable pin 13 SCLK pull-up – leave pull-up in place on the other 3 lines

    // Enable the SPI clock
    power_spi_enable();

    // Enable SPI peripheral
    SPCR = keep_SPCR;

    delay(10);
}

void turnOffSDcard()
{
    delay(6);

    // Disable SPI
    SPCR = 0;

    // Disable SPI clock
    power_spi_disable();

    // Set all SPI pins to INPUT
    DDRB &= ~((1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2));
    // Set all SPI pins HIGH
    PORTB |= ((1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2));

    // Note: LED_BUILTIN on pin 13 must be removed, otherwise it bleeds current

    // Wait 1 second for internal SD-card handling
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);

    // Turn SD-card power off via basis NPN transistor
    pinMode(SD_CARD_POWER_PIN, OUTPUT);
    digitalWrite(SD_CARD_POWER_PIN, LOW);
}

static void saveSdCard()
{
    char csvFilename[14];
    char csvTimestamp[25];
    char csvRow[40];
    char tempStr[10];
    bool writeHeader = false;
    File csvFile;

    // Create CSV filename
    snprintf(csvFilename, sizeof(csvFilename),
             "%d%02d.csv", dt.year, dt.month);

    // Get timestamp
    snprintf(csvTimestamp, sizeof(csvTimestamp),
             "%d-%02d-%02d, %d:%02d:00", 
             dt.year, dt.month, dt.dayMonth, 
             dt.hour, dt.minute);

    Serial.print(F("Saving "));
    Serial.print(csvFilename);
    Serial.print(F("..."));

    // Enable power to SD-card
    turnOnSDcard();

    // Re-init SD-card (ignore return value which is broken in SD library)
    (void)SD.begin(SPI_QUARTER_SPEED, SD_CARD_CS_PIN);

    // Check if file exists
    if (!SD.exists(csvFilename)) {
        writeHeader = true;
    }

    // Open file
    csvFile = SD.open(csvFilename, FILE_WRITE);
    if (!csvFile) {
        Serial.println(F("Error: Cannot open file"));
        ledBlink(5);
        return;
    }

    // Write CSV header if new file created
    if (writeHeader) {
        snprintf(csvRow, sizeof(csvRow), "Timestamp, Temperature\n");
        if (csvFile.write(csvRow, strlen(csvRow)) != strlen(csvRow)) {
            Serial.println(F("Error: Write header failed"));
            ledBlink(5);
            return;
        }
    }

    dtostrf(temperature, 4, 2, tempStr);
    snprintf(csvRow, sizeof(csvRow), "%s, %s\n", csvTimestamp, tempStr);
    if (csvFile.write(csvRow, strlen(csvRow)) != strlen(csvRow)) {
        Serial.println(F("Error: Write data failed"));
        ledBlink(6);
        return;
    } else {
        Serial.print(csvRow);
    }

    // Close file
    csvFile.close();

    // Remove power from SD-card
    turnOffSDcard();
}

void setup(void)
{
    char line[25];
    
    // Initialize serial
    Serial.begin(115200);
    Serial.println(F("Erriez DS1820 SD-card logger\n"));

    // Initialize LED
    ledInit();
    ledBlink(2);

    // Initialize TWI
    Wire.begin();
    Wire.setClock(400000);

    // Initialize RTC
#ifdef RTC_SET_DATE_TIME
    // Set new RTC date/time
    rtc.setDateTime(&dt);
#endif
    rtc.setAlarm1(Alarm1MatchSeconds, 0, 0, 0, 0);
    rtc.clearAlarmFlag(Alarm1);
    rtc.clearAlarmFlag(Alarm2);
    rtc.alarmInterruptEnable(Alarm1, true);
    rtc.alarmInterruptEnable(Alarm2, false);
    rtc.outputClockPinEnable(false);
    //rtc.setSquareWave(SquareWave1Hz);

    // Attach to INT0 interrupt falling edge
    pinMode(DS3231_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(DS3231_INT_PIN), rtcHandler, FALLING);

    // Initialize DS1820 temperature sensor
    ds1820.begin();
    ds1820.setResolution(12);
    ds1820.setWaitForConversion(false);
}

void loop(void)
{
    if (rtcInterrupt) {
        // Read date time from RTC
        ledOn();
        readRTCDateTime();
        Serial.flush();
        LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_ON);
        ledOff();

        // Read temperature
        readTemperature();
        
        if ((dt.minute % 10) == 0) {
            // Save to SD-card every 10 minutes
            ledOn();
            saveSdCard();
            ledOff();
        } else {
            // Print date time and temperature
            printDateTimeTemperature();
        }

        // Clear alarm interrupt
        rtc.clearAlarmFlag(Alarm1);

        rtcInterrupt = false;
    }

    // Wait for RTC interrupt
    Serial.flush();
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
}
