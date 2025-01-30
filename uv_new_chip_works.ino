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
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Adafruit_LTR390.h"

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_LTR390 ltr = Adafruit_LTR390();
float realtemp; 

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

#include "SparkFun_Weather_Meter_Kit_Arduino_Library.h"

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
  while (!Serial);
  Serial.println(F("BME680 async test"));

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

//being the uv data shit
  Serial.println("Adafruit LTR-390 test");

  if ( ! ltr.begin() ) {
    Serial.println("Couldn't find LTR sensor!");
    while (1) delay(10);
  }
  Serial.println("Found LTR sensor!");

  ltr.setMode(LTR390_MODE_UVS);
  if (ltr.getMode() == LTR390_MODE_ALS) {
    Serial.println("In ALS mode");
  } else {
    Serial.println("In UVS mode");
  }

  ltr.setGain(LTR390_GAIN_3);
  Serial.print("Gain : ");
  switch (ltr.getGain()) {
    case LTR390_GAIN_1: Serial.println(1); break;
    case LTR390_GAIN_3: Serial.println(3); break;
    case LTR390_GAIN_6: Serial.println(6); break;
    case LTR390_GAIN_9: Serial.println(9); break;
    case LTR390_GAIN_18: Serial.println(18); break;
  }

  ltr.setResolution(LTR390_RESOLUTION_16BIT);
  Serial.print("Resolution : ");
  switch (ltr.getResolution()) {
    case LTR390_RESOLUTION_13BIT: Serial.println(13); break;
    case LTR390_RESOLUTION_16BIT: Serial.println(16); break;
    case LTR390_RESOLUTION_17BIT: Serial.println(17); break;
    case LTR390_RESOLUTION_18BIT: Serial.println(18); break;
    case LTR390_RESOLUTION_19BIT: Serial.println(19); break;
    case LTR390_RESOLUTION_20BIT: Serial.println(20); break;
  }

  ltr.setThresholds(100, 1000);
  ltr.configInterrupt(true, LTR390_MODE_UVS);

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
    calibrationParams.vaneADCValues[WMK_ANGLE_0_0] = 0;
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
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  Serial.print(F("Reading started at "));
  Serial.print(millis());
  Serial.print(F(" and will finish at "));
  Serial.println(endTime);

  Serial.println(F("You can do other work during BME680 measurement."));
  delay(50); // This represents parallel work.
  // There's no need to delay() until millis() >= endTime: bme.endReading()
  // takes care of that. It's okay for parallel work to take longer than
  // BME680's measurement time.

  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  Serial.print(F("Reading completed at "));
  Serial.println(millis());

  Serial.print(F("Temperature = "));
  //.Serial.print(bme.temperature);
  //Serial.println(F(" *C"));
  realtemp = (bme.temperature*9)/5 +32; 
  Serial.print(realtemp); 
  Serial.println(F(" *F"));

  Serial.print(F("Pressure = "));
  Serial.print(bme.pressure / 100.0);
  Serial.println(F(" hPa"));

  Serial.print(F("Humidity = "));
  Serial.print(bme.humidity);
  Serial.println(F(" %"));

  Serial.print(F("Gas = "));
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(F(" KOhms"));

  /*Serial.print(F("Approx. Altitude = "));
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(F(" m"));*/

if (ltr.newDataAvailable()) {
      Serial.print("UV data: "); 
      Serial.print(ltr.readUVS());
      Serial.print("\n");
  }

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