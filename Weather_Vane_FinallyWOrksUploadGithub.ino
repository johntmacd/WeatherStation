/*************************************************** 
  This is an example for the LTR390 UV Sensor

  Designed specifically to work with the LTR390 UV sensor from Adafruit
  ----> https://www.adafruit.com

  These sensors use I2C to communicate, 2 pins are required to  
  interface
 ****************************************************/
 
#include "Adafruit_LTR390.h"
#include <Wire.h>
#include "SparkFunMPL3115A2.h"

Adafruit_LTR390 ltr = Adafruit_LTR390();

//Create an instance of the object
MPL3115A2 myPressure;

#define LEDPIN 11         //LED brightness (PWM) writing
#define LIGHTSENSORPIN A0 //Ambient light sensor reading 

int sensorValue;

//End of nicks code, start of clays 

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

//end of clays code before the void setup 

void setup() {
  Serial.begin(9600);
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

                                                                       // Above is the setup code for the UV light sensor  
                                                                       //Below is the code for the temp, pressure and altitude sensor


  Wire.begin();        // Join i2c bus

  myPressure.begin(); // Get sensor online

  //Configure the sensor
  myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa

  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
  pinMode(LIGHTSENSORPIN,  INPUT);  
  pinMode(LEDPIN, OUTPUT); 
  sensorValue = analogRead(0);       // read analog input pin 0
  Serial.println(sensorValue, DEC);  // prints the value read
  delay(100); 

  //end of nicks code in void setup, start of clays. 

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

    //end of clays code for void setup 

}

void loop() {
  if (ltr.newDataAvailable()) {
      Serial.print("UV data: "); 
      Serial.print(ltr.readUVS());
      Serial.print("\n");
  }
  
  delay(100);

                                              //above is the loop code for the UV data
                                              //below is the loop code for the Temp, altitude and pressure 

  float altitude = myPressure.readAltitudeFt();
  Serial.print("Altitude(ft):");
  Serial.print(altitude, 2);

  float temperature = myPressure.readTempF();
  Serial.print("\nTemp(f):");
  Serial.print(temperature, 2); 

  Serial.println();

  //end of nicks code for the void loop 

  // Print data from weather meter kit
    Serial.print(F("Wind direction (degrees): "));
    Serial.print(weatherMeterKit.getWindDirection(), 1);
    Serial.print(F("\t\t"));
    Serial.print(F("Wind speed (kph): "));
    Serial.print(weatherMeterKit.getWindSpeed(), 1);
    Serial.print(F("\t\t"));
    Serial.print(F("Total rainfall (mm): "));
    Serial.println(weatherMeterKit.getTotalRainfall(), 1);

    //end of clays code for the void setup 

  delay(1000);


}
