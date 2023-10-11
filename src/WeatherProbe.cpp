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


//MHZ-19 CO2 sensor
#define CO2_PWM 33
#define CO2_SWITCH 26
MHZ19 co2(CO2_PWM, GetTimeMS);
RTC_DATA_ATTR unsigned long CO2PreheatStartTime;
RTC_DATA_ATTR bool CO2Heating;

//PMS7003 PMS sensor
constexpr auto PMS_RX = 16;
constexpr auto PMS_TX = 17;
#define PMS_SWITCH 4
SerialPM pms(PMS7003, PMS_RX, PMS_TX); // PMSx003, RX, TX
RTC_DATA_ATTR bool PMSStabalizing;

//BME280 temperature/humidity/pressure sensor
Adafruit_BME280 bme; // I2C, 22 = SCL, 21 = SDA

#define BATT_LEVEL_PIN 36

#define USE_TASKS


#ifdef USE_TASKS

enum
{
  TASK_TEMP = 0,
  TASK_PM,
  TASK_CO2,
  TASK_BATT_LEVEL,
  TASK_COUNT
};
RTC_DATA_ATTR Task Tasks[TASK_COUNT];

#else

const unsigned int CO2Period = 1000 * 60 * 15;   //this is unlikely to change very frequently and takes a lot of energy to do
const unsigned int PMSPeriod = 1000 * 60;
const unsigned int TempPeriod = 1000 * 60;
const unsigned int PMSStabalizeTimeOffsetMS = 5000; //this seems about right in testing

RTC_DATA_ATTR unsigned long NextCO2ReadingTimeMS;
int LastCO2Reading = -1;
RTC_DATA_ATTR unsigned long NextPMSReadingTimeMS;
RTC_DATA_ATTR unsigned int PMSStabalizeTimeMS;

#endif

void TurnOnCO2()
{
  DebugPrintf("+++++++ Starting to heat CO2 +++++++\n");
  co2.ResetPreheatTime();
  digitalWrite(CO2_SWITCH, HIGH);
#ifdef USE_TASKS
  CO2Heating = true;
  CO2PreheatStartTime = GetTimeMS();  
#endif
}

void TurnOnPMS()
{
  DebugPrintf("+++++++ Turning on PM sensor +++++++\n");
#ifndef USE_TASKS
  PMSStabalizeTimeMS = GetTimeMS() + PMSStabalizeTimeOffsetMS;
  PMSStabalizing = true;
#endif
  digitalWrite(PMS_SWITCH, HIGH);
}

//runs ONLY ONCE
void ActualSetup()
{
#ifdef USE_TASKS
  TaskInit(Tasks[TASK_TEMP], 30      * 1000, 0);
  TaskInit(Tasks[TASK_PM],   30 * 60 * 1000, 5      * 1000);
  //CO2 starts outputting at @20 seconds (after short off time)
  //maybe real data starts coming in after 1:20 (80s)
  //no change at 3:00
  //long cool down vs short cooldown seems to make no difference in behavior
  TaskInit(Tasks[TASK_CO2],  60 * 60 * 1000, 3 * 60 * 1000);
  TaskInit(Tasks[TASK_BATT_LEVEL], 60 * 60 * 1000, 5 * 1000);

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

  for (int i = 0; i < TASK_COUNT; i++)
    TaskTick(Tasks[i]);

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

#else
  NextCO2ReadingTimeMS = GetTimeMS();
  NextPMSReadingTimeMS = GetTimeMS();
#endif

  //turn on sensors to start
  TurnOnCO2();
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

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  DebugPrint("Wakeup cause = " + String(wakeup_reason) + "\n");
  DebugPrint("Boot time = " + String(millis()) + "; now = " + String(GetTimeMS()) + "\n");

  if (wakeup_reason < 1 || wakeup_reason > 5)
  {
    ActualSetup();
  }
#ifdef USE_TASKS
  DebugPrintf("Next CO2 time = %u, next PM time = %u\n", TaskGetNextEventTime(Tasks[TASK_CO2]), TaskGetNextEventTime(Tasks[TASK_PM]));
#else
  DebugPrintf("Next CO2 time = %u, next PM time = %u\n", NextCO2ReadingTimeMS, NextPMSReadingTimeMS);
#endif

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

  DebugPrint("Nothing to do for MHZ-19 CO2 sensor!\n");
#ifdef USE_TASKS
#else
  if (CO2Heating)
    co2.SetPreheatStartTime(CO2PreheatStartTime);
#endif
  DebugPrint("Sensors initialized!\n");
}

void DoTaskTemp(int state)
{
  if (state == 2)
  {
    float temp = bme.readTemperature();
    float humid = bme.readHumidity();
    float pressure = bme.readPressure();
    DebugPrintf(" ========= temp = %.2f, humid = %.2f, pressure = %.2f\n", temp, humid, pressure);
    //TODO: communicate this
    return;
  }
}

void DoTaskPM(int state)
{
  if (state == 1) //start warmup
  {
    TurnOnPMS();
    pms.init();
    return;
  }

  if (state == 2) //take reading
  {
    DebugPrintf(" ---- TAKING PM READING ----\n");
    pms.read();
    digitalWrite(PMS_SWITCH, LOW);
    if (pms)
    {
      DebugPrintf(" ======= PM1.0 %hu, PM2.5 %hu, PM10 %hu [ug/m3]\n", pms.pm01, pms.pm25, pms.pm10);
      //TODO: Communicate this
    }
    else
    {
      DebugPrintf("Failed to read PM\n");
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
    return;
  }
}

void DoTaskCO2(int state)
{
  if (state == 1) //start warmup
  {
    TurnOnCO2();
    return;
  }
  if (state == 2) //take reading
  {
    DebugPrintf(" ----- Taking CO2 Reading -----\n");
    int co2Reading = co2.GetCO2();
    digitalWrite(CO2_SWITCH, LOW);
    DebugPrintf(" ========= CO2: %d\n", co2Reading);
    //TODO: Communicate this
    return;
  }
}

void DoTaskBattLevel(int state)
{
  if (state == 2) //take reading
  {
    int reading = analogRead(BATT_LEVEL_PIN);
    float voltage = reading;
    voltage /= 4096.0;
    voltage *= 2.5; // 5 / 2
    DebugPrintf("BATTERY READING = %d -> %.1f Volts\n", reading, voltage);
    //TODO: Communicate this
    return;
  }
}

void loop()
{
#ifdef USE_TASKS
  for (int i = 0; i < TASK_COUNT; i++)
  {
    Task& task = Tasks[i];
    int result = TaskTick(task);
    if (result == 0)
      continue;
    switch (i)
    {
      case TASK_TEMP:
        DoTaskTemp(result);
        break;
      case TASK_PM:
        DoTaskPM(result);
        break;
      case TASK_CO2:
        DoTaskCO2(result);
        break;
      case TASK_BATT_LEVEL:
        DoTaskBattLevel(result);
        break;
    }
  }
#else
  //do CO2 things...
  if (GetTimeMS() - NextCO2ReadingTimeMS < 0x7FFFFFFF)
  {
    DebugPrintf("Time to take a CO2 reading: %u > %u\n", GetTimeMS(), NextCO2ReadingTimeMS);
    if (!CO2Heating)
    {
      TurnOnCO2();
    }
    if (co2.IsPreheated())
    {
      LastCO2Reading = co2.GetCO2();
      DebugPrintf(" ======= co2 = %d\n", LastCO2Reading);
      NextCO2ReadingTimeMS += CO2Period;
      digitalWrite(CO2_SWITCH, LOW);
      CO2Heating = false;
    }
    else
    {
      DebugPrintf("CO2 is still heating...\n");
    }
  }

  //do PMS things...
  if (GetTimeMS() - NextPMSReadingTimeMS < 0x7FFFFFFF)
  {
    DebugPrintf("Time to take a PM reading: %u > %u\n", GetTimeMS(), NextPMSReadingTimeMS);
    if (!PMSStabalizing)
    {
      TurnOnPMS();
    }
    if (GetTimeMS() - PMSStabalizeTimeMS < 0x7FFFFFFF)
    {
      pms.init();
      pms.read();
      if (pms)
      {
        DebugPrintf(" ======= PM1.0 %hu, PM2.5 %hu, PM10 %hu [ug/m3]\n", pms.pm01, pms.pm25, pms.pm10);
      }
      else
      {
        DebugPrint("Failed to read\n");
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
      NextPMSReadingTimeMS += PMSPeriod;
      digitalWrite(PMS_SWITCH, LOW);
      PMSStabalizing = false;
    }
  }

  float temp = bme.readTemperature();
  float humid = bme.readHumidity();
  float pressure = bme.readPressure();
  DebugPrintf("temp = %.2f, humid = %.2f, pressure = %.2f\n", temp, humid, pressure);

  //TODO: communicate the current readings

  esp_sleep_enable_timer_wakeup(1000 * 1000); //in microseconds
#endif

#ifdef USE_TASKS
  unsigned long earliestEvent = TaskGetNextEventTime(Tasks[0]);
  DebugPrintf("Task 0 next at %u\n", earliestEvent);
  bool goToSleep = true;
  for (int i = 1; i < 3; i++)
  {
    unsigned long nextEvent = TaskGetNextEventTime(Tasks[i]);
    DebugPrintf("Task %d next at %u, state = %d\n", i, nextEvent, Tasks[i].m_State);
    if (earliestEvent - nextEvent < 0x7FFFFFFF)
      earliestEvent = nextEvent;
    goToSleep &= Tasks[i].m_State != Task::WARMING; //any warming will prevent sleeping
  }
  unsigned int sleepTimeMS = earliestEvent - GetTimeMS();
  DebugPrintf("Next event is at %u (in %u ms)\n", earliestEvent, sleepTimeMS);
  if (sleepTimeMS > 0x7FFFFFFF)
    return; //we are passed due somehow!

  if (!goToSleep)
  {
    DebugPrint("Delaying\n");
    delay(sleepTimeMS);
    return;
  }
  //time to go to sleep!
  DebugPrint("DEEP SLEEPING!\n");
  esp_sleep_enable_timer_wakeup(sleepTimeMS * 1000); //in microseconds

#endif

  DebugSleep();
  TimeGoToSleep();
  esp_deep_sleep_start();
}
