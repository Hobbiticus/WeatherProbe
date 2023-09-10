#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "mhz19.h"
#include <PMserial.h> // Arduino library for PM sensors with serial interface

#define CO2_PWM 25
MHZ19 co2(-1, -1, CO2_PWM);

//PMS7003 PMS sensor
constexpr auto PMS_RX = 17;
constexpr auto PMS_TX = 16;
SerialPM pms(PMS7003, PMS_RX, PMS_TX); // PMSx003, RX, TX

Adafruit_BME280 bme; // I2C

// put function declarations here:
int myFunction(int, int);

void setup()
{
  Serial.begin(115200);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  Serial.println("Starting pms...");
  pms.init();
  Serial.println("PMS started!");


  // int status = bme.begin(BME280_ADDRESS_ALTERNATE);  
  //   // You can also pass in a Wire library object like &Wire2
  //   // status = bme.begin(0x76, &Wire2)
  //   if (!status) {
  //       Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  //       Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
  //       Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
  //       Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
  //       Serial.print("        ID of 0x60 represents a BME 280.\n");
  //       Serial.print("        ID of 0x61 represents a BME 680.\n");
  //       while (1) delay(10);
  //   }
  //   else{
  //     Serial.println("yay!");
  //   }
}

void loop()
{
  // float temp = bme.readTemperature();
  // float humid = bme.readHumidity();
  // float pressure = bme.readPressure();
  // Serial.printf("temp = %.2f, humid = %.2f, pressure = %.2f\n", temp, humid, pressure);
  // if (!isnan(temp))
  // int co2_ppm = co2.GetCO2();  
  // Serial.printf("co2 = %d\n", co2_ppm);
  // if (true)

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

