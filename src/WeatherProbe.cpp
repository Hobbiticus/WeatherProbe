#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "mhz19.h"
#include <PMserial.h> // Arduino library for PM sensors with serial interface

//MHZ-19 CO2 sensor
#define CO2_PWM 33
#define CO2_SWITCH 26
MHZ19 co2(CO2_PWM);

//PMS7003 PMS sensor
constexpr auto PMS_RX = 17;
constexpr auto PMS_TX = 16;
#define PMS_SWITCH 4
SerialPM pms(PMS7003, PMS_RX, PMS_TX); // PMSx003, RX, TX

//BME280 temperature/humidity/pressure sensor
Adafruit_BME280 bme; // I2C, 22 = SCL, 21 = SDA

#define BATT_LEVEL_PIN 36


const unsigned int CO2Period = 1000 * 60 * 15;

unsigned int NextCO2ReadingTimeMS = 0;
int LastCO2Reading = -1;
int NextPMReadingTime = 0;

void setup()
{
  Serial.begin(115200);

  //setup pings
  pinMode(CO2_SWITCH, OUTPUT);
  pinMode(PMS_SWITCH, OUTPUT);
  pinMode(BATT_LEVEL_PIN, INPUT_PULLUP);

  //enable all sensors to start
  digitalWrite(CO2_SWITCH, HIGH);
  digitalWrite(PMS_SWITCH, HIGH);
  NextCO2ReadingTimeMS = millis();
  co2.ResetPreheatTime();

  Serial.println("Starting pms...");
  pms.init();
  Serial.println("PMS started!");

  Serial.println("Starting BME280 temperature/humidity/pressure sensor...");
  int status = bme.begin(BME280_ADDRESS_ALTERNATE);  
  // You can also pass in a Wire library object like &Wire2
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }
  else{
    Serial.println("yay!");
  }

  Serial.println("Nothing to do for MHZ-19 CO2 sensor!");
  Serial.println("Sensors initialized!");
}

bool CO2Heating = true;
void loop()
{
  //do CO2 things...
  if (millis() > NextCO2ReadingTimeMS)
  {
    if (!CO2Heating)
    {
      co2.ResetPreheatTime();
      digitalWrite(CO2_SWITCH, HIGH);
      CO2Heating = true;
    }
    if (co2.IsPreheated())
    {
      LastCO2Reading = co2.GetCO2();
      Serial.printf("co2 = %d\n", LastCO2Reading);
      NextCO2ReadingTimeMS += CO2Period;
      //TODO: do something here to output the new co2 reading
      digitalWrite(CO2_SWITCH, LOW);
    }
  }

  float temp = bme.readTemperature();
  float humid = bme.readHumidity();
  float pressure = bme.readPressure();
  Serial.printf("temp = %.2f, humid = %.2f, pressure = %.2f\n", temp, humid, pressure);

  pms.read();
  if (pms)
  {
    digitalWrite(2, HIGH);
    Serial.printf("PM1.0 %hu, PM2.5 %hu, PM10 %hu [ug/m3]\n", pms.pm01, pms.pm25, pms.pm10);
  }
  else
  {
    digitalWrite(2, LOW);
    Serial.println("Failed to read");
    switch (pms.status)
    {
    case pms.OK: // should never come here
      break;     // included to compile without warnings
    case pms.ERROR_TIMEOUT:
      Serial.println(F(PMS_ERROR_TIMEOUT));
      break;
    case pms.ERROR_MSG_UNKNOWN:
      Serial.println(F(PMS_ERROR_MSG_UNKNOWN));
      break;
    case pms.ERROR_MSG_HEADER:
      Serial.println(F(PMS_ERROR_MSG_HEADER));
      break;
    case pms.ERROR_MSG_BODY:
      Serial.println(F(PMS_ERROR_MSG_BODY));
      break;
    case pms.ERROR_MSG_START:
      Serial.println(F(PMS_ERROR_MSG_START));
      break;
    case pms.ERROR_MSG_LENGTH:
      Serial.println(F(PMS_ERROR_MSG_LENGTH));
      break;
    case pms.ERROR_MSG_CKSUM:
      Serial.println(F(PMS_ERROR_MSG_CKSUM));
      break;
    case pms.ERROR_PMS_TYPE:
      Serial.println(F(PMS_ERROR_PMS_TYPE));
      break;
    }
  }
  delay(1000);
}
