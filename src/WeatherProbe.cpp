#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "mhz19.h"
#include <PMserial.h> // Arduino library for PM sensors with serial interface
#include "debug.h"
#include "MyTime.h"
#include "Tasker.h"
#include "EspNowRelay.h"
#include "../../CentralBrain/include/IngestProtocol.h"
#include "../../CentralBrain/include/WeatherProtocol.h"

unsigned char RelayMAC[6] = {0xC8, 0xC9, 0xA3, 0xD2, 0x9D, 0xC8};
unsigned char BrainAddr[4] = {192, 168, 1, 222};
unsigned char PiAddr[4] = {192, 168, 1, 135}; //maybe? might be wrong

const unsigned short IngestPort = 7777;


//MHZ-19 CO2 sensor
#define CO2_PWM 33
#define CO2_SWITCH 26
#define CO2_RX 35
#define CO2_TX 25
//MHZ19 co2(CO2_PWM, GetTimeMS);
MHZ19 co2(CO2_RX, CO2_TX, GetTimeMS);

//PMS7003 PMS sensor
constexpr auto PMS_RX = 16;
constexpr auto PMS_TX = 17;
#define PMS_SWITCH 4
SerialPM pms(PMS7003, PMS_RX, PMS_TX); // PMSx003, RX, TX
RTC_DATA_ATTR bool PMSStabalizing;

//BME280 temperature/humidity/pressure sensor
Adafruit_BME280 bme; // I2C, 22 = SCL, 21 = SDA

#define BATT_LEVEL_PIN 36

EspNowRelay NowRelay;

enum
{
  TASK_TEMP = 0,
  //TASK_CO2,
  TASK_PM,
  TASK_BATT_LEVEL,
  TASK_COUNT
};
RTC_DATA_ATTR Task Tasks[TASK_COUNT];

// void TurnOnCO2()
// {
//   DebugPrintf("+++++++ Starting to heat CO2 +++++++\n");
//   digitalWrite(CO2_SWITCH, HIGH);
//   co2.ResetPreheatTime();
// }

void TurnOnPMS()
{
  DebugPrintf("+++++++ Turning on PM sensor +++++++\n");
  digitalWrite(PMS_SWITCH, HIGH);
  pms.init();
}

bool DoTaskTemp(int state, TemperatureData& data)
{
  if (state == 2)
  {
    float temp = bme.readTemperature();
    float humid = bme.readHumidity();
    float pressure = bme.readPressure();
    DebugPrintf(" ========= temp = %.2f, humid = %.2f, pressure = %.2f\n", temp, humid, pressure);
    data.m_Temperature = (short)(temp * 100);
    data.m_Humidity = (unsigned short)(humid * 10);
    data.m_Pressure = (unsigned int)(pressure * 100);
    return true;
  }
  return false;
}

bool DoTaskPM(int state, PMData& data)
{
  if (state == 1) //start warmup
  {
    TurnOnPMS();
    return false;
  }

  if (state == 2) //take reading
  {
    DebugPrintf(" ---- TAKING PM READING ----\n");
    pms.read();
    digitalWrite(PMS_SWITCH, LOW);
    if (pms)
    {
      DebugPrintf(" ======= PM1.0 %hu, PM2.5 %hu, PM10 %hu [ug/m3]\n", pms.pm01, pms.pm25, pms.pm10);
      data.m_10 = pms.pm10;
      data.m_2_5 = pms.pm25;
      data.m_0_1 = pms.pm01;
      return true;
    }
    else
    {
      DebugPrintf("Failed to read PM\n");
    }
  }
  return false;
}

// bool DoTaskCO2(int state, CO2Data& data)
// {
//   if (state == 1) //start warmup
//   {
//     TurnOnCO2();
//     return false;
//   }
//   if (state == 2) //take reading
//   {
//     DebugPrintf(" ----- Taking CO2 Reading -----\n");
//     int co2Reading = co2.GetCO2();
//     digitalWrite(CO2_SWITCH, LOW);
//     DebugPrintf(" ========= CO2: %d\n", co2Reading);
//     data.m_PPM = co2Reading;
//     return true;
//   }

//   return false;
// }

bool DoTaskBattLevel(int state, BatteryData& data)
{
  if (state == 2) //take reading
  {
    int reading = analogRead(BATT_LEVEL_PIN);
    float voltage = reading;
    voltage /= 4096.0;
    voltage *= 8.48;
    DebugPrintf("BATTERY READING = %d -> %.2f Volts\n", reading, voltage);
    data.m_Voltage = (unsigned int)(voltage * 100);
    return true;
  }
  return false;
}

void ExecuteTasks()
{
  unsigned char out[256] = {0};
  IngestHeader* ih = (IngestHeader*)out;
  ih->m_Type = DATA_TYPE_WEATHER;
  WeatherHeader* wh = (WeatherHeader*)(ih + 1);
  wh->m_DataIncluded = 0;
  unsigned char* ptr = (unsigned char*)(wh + 1);

  for (int i = 0; i < TASK_COUNT; i++)
  {
    Task& task = Tasks[i];
    int result = TaskTick(task);
    if (result == 0)
      continue;
    switch (i)
    {
      case TASK_TEMP:
      {
        TemperatureData* data = (TemperatureData*)ptr;
        if (DoTaskTemp(result, *data))
        {
          ptr += sizeof(TemperatureData);
          wh->m_DataIncluded |= WEATHER_TEMP_BIT;
        }
        break;
      }
      // case TASK_CO2:
      // {
      //   CO2Data* data = (CO2Data*)ptr;
      //   if (DoTaskCO2(result, *data))
      //   {
      //     ptr += sizeof(CO2Data);
      //     wh->m_DataIncluded |= WEATHER_CO2_BIT;
      //   }
      //   break;
      // }
      case TASK_PM:
      {
        PMData* data = (PMData*)ptr;
        if (DoTaskPM(result, *data))
        {
          ptr += sizeof(PMData);
          wh->m_DataIncluded |= WEATHER_PM_BIT;
        }
        break;
      }
      case TASK_BATT_LEVEL:
      {
        BatteryData* data = (BatteryData*)ptr;
        if (DoTaskBattLevel(result, *data))
        {
          ptr += sizeof(BatteryData);
          wh->m_DataIncluded |= WEATHER_BATT_BIT;
        }
        break;
      }
    }
  }

  if (wh->m_DataIncluded != 0)
  {
    //send the data to the central brain
    unsigned int packetLen = (unsigned int)(ptr - out);

    Serial.println("Connecting...");
    int sock = NowRelay.Connect(*((unsigned int*)BrainAddr), IngestPort);
    Serial.printf("Connect to brain returned %d\n", sock);
    if (sock >= 0)
    {
      NowRelay.Send(sock, out, packetLen);
      NowRelay.Close(sock);
    }
  }
}

//runs ONLY ONCE
void ActualSetup()
{
  TaskInit(Tasks[TASK_TEMP], 30 * 1000,      0);
  //CO2 starts outputting at @20 seconds (after short off time)
  //maybe real data starts coming in after 1:20 (80s)
  //no change at 3:00
  //long cool down vs short cooldown seems to make no difference in behavior
  //TaskInit(Tasks[TASK_CO2],  30 * 60 * 1000, 182 * 1000);
  TaskInit(Tasks[TASK_PM],   10 * 60 * 1000, 5      * 1000);
  TaskInit(Tasks[TASK_BATT_LEVEL], 1 * 60 * 1000, 0);

  {
    unsigned long earliestEvent = TaskGetNextEventTime(Tasks[0]);
    DebugPrintf("Task 0 next at %u\n", earliestEvent);
    for (int i = 1; i < TASK_COUNT; i++)
    {
      unsigned long nextEvent = TaskGetNextEventTime(Tasks[i]);
      DebugPrintf("Task %d next at %u, state = %d\n", i, nextEvent, Tasks[i].m_State);
      if (earliestEvent - nextEvent < 0x7FFFFFFF)
        earliestEvent = nextEvent;
    }
    unsigned int sleepTimeMS = earliestEvent - GetTimeMS();
    DebugPrintf("Next event is at %u (in %u ms)\n", earliestEvent, sleepTimeMS);
  }

  ExecuteTasks();

  DebugPrint("After initial tick....\n");
  {
    unsigned long earliestEvent = TaskGetNextEventTime(Tasks[0]);
    DebugPrintf("Task 0 next at %u\n", earliestEvent);
    for (int i = 1; i < TASK_COUNT; i++)
    {
      unsigned long nextEvent = TaskGetNextEventTime(Tasks[i]);
      DebugPrintf("Task %d next at %u, state = %d\n", i, nextEvent, Tasks[i].m_State);
      if (earliestEvent - nextEvent < 0x7FFFFFFF)
        earliestEvent = nextEvent;
    }
    unsigned int sleepTimeMS = earliestEvent - GetTimeMS();
    DebugPrintf("Next event is at %u (in %u ms)\n", earliestEvent, sleepTimeMS);
  }

  //turn on sensors to start
  //TurnOnCO2();
  TurnOnPMS();
}

//runs every time we boot
void setup()
{
  Serial.begin(115200);

  //setup pings
  pinMode(CO2_SWITCH, OUTPUT);
  pinMode(PMS_SWITCH, OUTPUT);
  pinMode(BATT_LEVEL_PIN, INPUT_PULLUP);

  DebugInit();

  NowRelay.Init(RelayMAC);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  DebugPrint("Wakeup cause = " + String(wakeup_reason) + "\n");
  DebugPrint("Boot time = " + String(millis()) + "; now = " + String(GetTimeMS()) + "\n");

  DebugPrint("Starting BME280 temperature/humidity/pressure sensor...\n");
  int status = bme.begin(BME280_ADDRESS_ALTERNATE);  
  // You can also pass in a Wire library object like &Wire2
  if (!status) {
      DebugPrint("Could not find a valid BME280 sensor, check wiring, address, sensor ID!\n");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      //while (1) delay(10);
  }
  else{
    DebugPrint("yay!\n");
  }

  if (wakeup_reason < 1 || wakeup_reason > 5)
  {
    ActualSetup();
  }
  //DebugPrintf("Next CO2 time = %u, next PM time = %u\n", TaskGetNextEventTime(Tasks[TASK_CO2]), TaskGetNextEventTime(Tasks[TASK_PM]));

  DebugPrint("Nothing to do for MHZ-19 CO2 sensor!\n");
  DebugPrint("Sensors initialized!\n");
}

void loop()
{
  ExecuteTasks();

  unsigned long earliestEvent = TaskGetNextEventTime(Tasks[0]);
  DebugPrintf("Task 0 next at %u\n", earliestEvent);
  bool goToSleep = true;
  for (int i = 1; i < TASK_COUNT; i++)
  {
    unsigned long nextEvent = TaskGetNextEventTime(Tasks[i]);
    DebugPrintf("Task %d next at %u, state = %d\n", i, nextEvent, Tasks[i].m_State);
    if (earliestEvent - nextEvent < 0x7FFFFFFF)
      earliestEvent = nextEvent;
    goToSleep &= Tasks[i].m_State != Task::WARMING; //any warming will prevent sleeping
  }
  unsigned long now = GetTimeMS();
  unsigned int sleepTimeMS = earliestEvent - now;
  if (earliestEvent < now)
    sleepTimeMS = 0;
  DebugPrintf("Next event is at %u (in %u ms)\n", earliestEvent, sleepTimeMS);
  if (sleepTimeMS > 0x7FFFFFFF)
    return; //we are passed due somehow!

  if (!goToSleep)
  {
    DebugPrintf("Delaying for %u ms\n", sleepTimeMS);
    delay(sleepTimeMS);
    return;
  }
  //time to go to sleep!
  DebugPrintf("DEEP SLEEPING! for %u ms\n", sleepTimeMS);
  esp_sleep_enable_timer_wakeup(sleepTimeMS * 1000); //in microseconds

  DebugSleep();
  TimeGoToSleep(sleepTimeMS);
  esp_deep_sleep_start();
}
