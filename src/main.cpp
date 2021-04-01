#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include "ESP32_OTA.h"
#include "luainc.h"

static lua_State *L = luaL_newstate();
const char* errmsg = NULL;


//=======

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

unsigned long delayTime;


void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Booting");
    setupWPS_OTA("LRH_Workshop");


    if (! bme.begin(0x76, &Wire)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }


    Serial.println("-- Weather Station Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );

    // suggested rate is 1/60Hz (1m)
    delayTime = 60000; // in milliseconds

}

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}


void loop() {
  // put your main code here, to run repeatedly:

    bme.takeForcedMeasurement(); // has no effect in normal mode

    printValues();
    delay(delayTime);

}