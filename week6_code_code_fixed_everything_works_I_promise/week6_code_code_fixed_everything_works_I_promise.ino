/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>

#include "Modulino.h"
#include "SparkFun_Weather_Meter_Kit_Arduino_Library.h"
#include "Arduino_LED_Matrix.h"

ModulinoThermo thermo;
ArduinoLEDMatrix matrix;

float temperature = -273.15;
float humidity = 0.0;

// Below are the pin definitions for each sensor of the weather meter kit

// Pins for Weather Carrier with ESP32 Processor Board
int windDirectionPin = A3;
int windSpeedPin = 3;
int rainfallPin = 2;

// Pins for the Weather Shield with SparkFun RedBoard Qwiic or Arduino Uno
// int windDirectionPin = A0;
// int windSpeedPin = 3;
// int rainfallPin = 2;

// Create an instance of the weather meter kit
SFEWeatherMeterKit weatherMeterKit(windDirectionPin, windSpeedPin, rainfallPin);

void setup() {
  Serial.begin(9600);

  Modulino.begin();
  thermo.begin();

  matrix.begin();
  delay(100);
  

    Serial.println(F("SparkFun Weather Meter Kit Example 2 - Manual Calibration"));
    Serial.println();
    Serial.println(F("Note - this example demonstrates how to manually set the"));
    Serial.println(F("calibration parameters once you know what they are for your"));
    Serial.println(F("set up. If you don't know what values to use, check out"));
    Serial.println(F("Example 3, which walks you through it! The values used in"));
    Serial.println(F("this example are all defaults, so you may need to change them."));

    // Here we create a struct to hold all the calibration parameters
    SFEWeatherMeterKitCalibrationParams calibrationParams = weatherMeterKit.getCalibrationParams();
    
    // The wind vane has 8 switches, but 2 could close at the same time, which
    // results in 16 possible positions. Each position has a resistor connected
    // to GND, so this library assumes a voltage divider is created by adding
    // another resistor to VCC. Some of the wind vane resistor values are
    // fairly close to each other, meaning an accurate ADC is required. However
    // some ADCs have a non-linear behavior that causes this measurement to be
    // inaccurate. To account for this, the vane resistor values can be manually
    // changed here to compensate for the non-linear behavior of the ADC
    /*calibrationParams.vaneADCValues[WMK_ANGLE_0_0] = 0;
    calibrationParams.vaneADCValues[WMK_ANGLE_22_5] = 64;
    calibrationParams.vaneADCValues[WMK_ANGLE_45_0] = 128;
    calibrationParams.vaneADCValues[WMK_ANGLE_67_5] = 192;
    calibrationParams.vaneADCValues[WMK_ANGLE_90_0] = 256;
    calibrationParams.vaneADCValues[WMK_ANGLE_112_5] = 320;
    calibrationParams.vaneADCValues[WMK_ANGLE_135_0] = 384;
    calibrationParams.vaneADCValues[WMK_ANGLE_157_5] = 448;
    calibrationParams.vaneADCValues[WMK_ANGLE_180_0] = 512;
    calibrationParams.vaneADCValues[WMK_ANGLE_202_5] = 576;
    calibrationParams.vaneADCValues[WMK_ANGLE_225_0] = 640;
    calibrationParams.vaneADCValues[WMK_ANGLE_247_5] = 704;
    calibrationParams.vaneADCValues[WMK_ANGLE_270_0] = 768;
    calibrationParams.vaneADCValues[WMK_ANGLE_292_5] = 832;
    calibrationParams.vaneADCValues[WMK_ANGLE_315_0] = 896;
    calibrationParams.vaneADCValues[WMK_ANGLE_337_5] = 960;
    */
    calibrationParams.vaneADCValues[WMK_ANGLE_0_0] = 788;
    calibrationParams.vaneADCValues[WMK_ANGLE_22_5] = 407;
    calibrationParams.vaneADCValues[WMK_ANGLE_45_0] = 562;
    calibrationParams.vaneADCValues[WMK_ANGLE_67_5] = 84;
    calibrationParams.vaneADCValues[WMK_ANGLE_90_0] = 93;
    calibrationParams.vaneADCValues[WMK_ANGLE_112_5] = 66;
    calibrationParams.vaneADCValues[WMK_ANGLE_135_0] = 185;
    calibrationParams.vaneADCValues[WMK_ANGLE_157_5] = 126;
    calibrationParams.vaneADCValues[WMK_ANGLE_180_0] = 288;
    calibrationParams.vaneADCValues[WMK_ANGLE_202_5] = 245;
    calibrationParams.vaneADCValues[WMK_ANGLE_225_0] = 632;
    calibrationParams.vaneADCValues[WMK_ANGLE_247_5] = 601;
    calibrationParams.vaneADCValues[WMK_ANGLE_270_0] = 948;
    calibrationParams.vaneADCValues[WMK_ANGLE_292_5] = 829;
    calibrationParams.vaneADCValues[WMK_ANGLE_315_0] = 889;
    calibrationParams.vaneADCValues[WMK_ANGLE_337_5] = 704;

    // The rainfall detector contains a small cup that collects rain water. When
    // the cup fills, the water is dumped and the total rainfall is incremented
    // by some value. This value defaults to 0.2794mm of rain per count, as
    // specified by the datasheet
    calibrationParams.mmPerRainfallCount = 0.2794;

    // The rainfall detector switch can sometimes bounce, causing multiple extra
    // triggers. This input is debounced by ignoring extra triggers within a
    // time window, which defaults to 100ms
    calibrationParams.minMillisPerRainfall = 100;

    // The anemometer contains a switch that opens and closes as it spins. The
    // rate at which the switch closes depends on the wind speed. The datasheet
    // states that a wind of 2.4kph causes the switch to close once per second
    calibrationParams.kphPerCountPerSec = 2.4;

    // Because the anemometer generates discrete pulses as it rotates, it's not
    // possible to measure the wind speed exactly at any point in time. A filter
    // is implemented in the library that averages the wind speed over a certain
    // time period, which defaults to 1 second. Longer intervals result in more
    // accurate measurements, but cause delay in the measurement
    calibrationParams.windSpeedMeasurementPeriodMillis = 1000;

    // Now we can set all the calibration parameters at once
    weatherMeterKit.setCalibrationParams(calibrationParams);

    // Begin weather meter kit
    weatherMeterKit.begin();

}

void loop() {
  //Acquire temperature and humidity
  temperature = (thermo.getTemperature() * 9 / 5) + 32;
  humidity = thermo.getHumidity();

  //Convert the temperature float to a string with 1 decimal point shown
  //and add °C at the end
  String temperature_text = String(temperature, 1) + "°F";

  //Convert the humidity float to a string with no decimal points shown
  //and add % at the end
  String humidity_text = String(humidity, 0) + "%";

  //Print each of the sensor values on serial
  Serial.print("Temperature ");
  Serial.print(temperature_text + "         ");
  Serial.print("Humidity ");
  Serial.println(humidity_text);

  //Show on the UNO R4 WiFi LED matrix the data

 
  
    Serial.print(F("Wind direction (degrees): "));
    Serial.print(weatherMeterKit.getWindDirection(), 1);
    Serial.print(F("\t\t"));
    Serial.print(F("Wind speed (kph): "));
    Serial.print(weatherMeterKit.getWindSpeed(), 1);
    Serial.print(F("\t\t"));
    Serial.print(F("Total rainfall (mm): "));
    Serial.println(weatherMeterKit.getTotalRainfall(), 1);


  Serial.println();
  delay(1000);
}