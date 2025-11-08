#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include "Adafruit_SHT31.h"
#include <PMserial.h> // Arduino library for PM sensors with serial interface
#include "INA236.h"
#include "debug.h"
#include "MyTime.h"
#include "Tasker.h"
#include "EspNowRelay.h"
#include "../../CentralBrain/include/IngestProtocol.h"
#include "../../CentralBrain/include/WeatherProtocol.h"

//BMP280 pressure sensor
Adafruit_BMP280 bmp;
//SHT31 temerature/humidity sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();
//INA236 voltage/current sensor
INA236 INA(0x40);

unsigned char RelayMAC[6] = {0xC8, 0xC9, 0xA3, 0xD2, 0x9D, 0xC8};
unsigned char BrainAddr[4] = {192, 168, 1, 222};

const unsigned short IngestPort = 7777;

//actual voltage, measured voltage
const int VoltageCalibrationTableSize = 5;
float VoltageCalibrationTable[VoltageCalibrationTableSize][2] =
{
  { 5.5, 5.01 },
  { 6.0, 5.52 },
  { 6.5, 6.05 },
  { 7.0, 6.69 },
  { 7.5, 7.50 }
};


//PMS7003 PMS sensor
constexpr auto PMS_RX = 16;
constexpr auto PMS_TX = 17;
#define PMS_SWITCH 27
SerialPM pms(PMS7003, PMS_RX, PMS_TX); // PMSx003, RX, TX
RTC_DATA_ATTR bool PMSStabalizing;

EspNowRelay NowRelay;

enum
{
  TASK_BATT_LEVEL = 0,
  TASK_TEMP,
  TASK_PM,
  TASK_COUNT
};
RTC_DATA_ATTR Task Tasks[TASK_COUNT];

void TurnOnPMS()
{
  DebugPrintf("+++++++ Turning on PM sensor +++++++\n");
  digitalWrite(PMS_SWITCH, HIGH);
  pms.init();
}

bool DoTaskTemp(int state, TemperatureData& data, unsigned char& bitsIncluded)
{
  if (state == 2)
  {
    DebugPrintf("sensor id: %u\n", bmp.sensorID());
    if (bmp.sensorID() == 0)
      return false;
    
    float temp = sht31.readTemperature(); //celcius
    float humid = sht31.readHumidity();   // %
    float pressure = bmp.readPressure();  // Pa
    DebugPrintf(" ========= temp = %.2f, humid = %.2f, pressure = %.2f\n", temp, humid, pressure);
    //validate for wonky readings
    if (temp != NAN && temp != 0)
    {
      bitsIncluded |= WEATHER_TEMP_ONLY_BIT;
      data.m_Temperature = (short)(temp * 100);
    }
    if (humid != NAN)
    {
      bitsIncluded |= WEATHER_HUMID_BIT;
      data.m_Humidity = (unsigned short)(humid * 10);
    }
    if (pressure != NAN && pressure < 118524 && pressure > 84659)
    {
      bitsIncluded |= WEATHER_PRESSURE_BIT;
      data.m_Pressure = (unsigned int)(pressure * 100);
    }
    //not sure if this ever happens, but hopefully this will prevent wonky readings
    if (temp == NAN || humid == NAN)
    {
      DebugPrintf("Temp or humidity is NAN\n");
      return false;
    }
    if (pressure == NAN)
    {
      DebugPrintf("Pressure is NAN\n");
      return false;
    }
    if (pressure > 118524) //35 inhg
    {
      DebugPrintf("  [!] Pressure is too high\n");
      return false;
    }
    if (pressure < 84659) //25 inhg
    {
      DebugPrintf("  [!] Pressure is to low\n");
      return false;
    }

    return true;
  }
  return false;
}

bool DoTaskPM(int state, PMData& data)
{
  if (state == 1) //start warmup
  {
    DebugPrintf(" --- Turning on PM sensor ---\n");
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

bool DoTaskBattLevel(int state, BatteryData& data)
{
  if (state == 2) //take reading
  {
    if (!INA.isConnected())
    {
      DebugPrintf("[!] Not connected to INA236\n");
      return false;
    }
    float voltage = INA.getBusVoltage();
    float current = INA.getCurrent();

    DebugPrintf(" ========= BATTERY READING = %.2f Volts, %.3f Amps\n", voltage, current);
    if (voltage == 0)
    {
      DebugPrintf("No voltage?\n");
      return false;
    }
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

  TemperatureData tempData;
  PMData pmData;
  BatteryData battData;

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
        DebugPrintf("Doing temperature task...\n");
        if (DoTaskTemp(result, tempData, wh->m_DataIncluded))
          wh->m_DataIncluded |= WEATHER_TEMP_BIT;
        break;
      }
      case TASK_PM:
      {
        DebugPrintf("Doing PM task...\n");
        if (DoTaskPM(result, pmData))
          wh->m_DataIncluded |= WEATHER_PM_BIT;
        break;
      }
      case TASK_BATT_LEVEL:
      {
        DebugPrintf("Doing Battery task...\n");
        if (DoTaskBattLevel(result, battData))
          wh->m_DataIncluded |= WEATHER_BATT_BIT;
        break;
      }
    }
  }

  if (wh->m_DataIncluded & WEATHER_TEMP_BIT)
  {
    TemperatureData* data = (TemperatureData*)ptr;
    *data = tempData;
    ptr += sizeof(TemperatureData);
  }
  if (wh->m_DataIncluded & WEATHER_PM_BIT)
  {
    PMData* data = (PMData*)ptr;
    *data = pmData;
    ptr += sizeof(PMData);
  }
  if (wh->m_DataIncluded & WEATHER_BATT_BIT)
  {
    BatteryData* data = (BatteryData*)ptr;
    *data = battData;
    ptr += sizeof(BatteryData);
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
  TaskInit(Tasks[TASK_TEMP], 30 * 1000, 0);
  TaskInit(Tasks[TASK_PM],   10 * 60 * 1000, 5 * 1000);
  TaskInit(Tasks[TASK_BATT_LEVEL], 2 * 60 * 1000, 0);

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

  DebugPrintf("After initial tick....\n");
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
  TurnOnPMS();
}

//runs every time we boot
void setup()
{
  Serial.begin(115200);

  //setup pins
  pinMode(PMS_SWITCH, OUTPUT);

  DebugInit();

  NowRelay.Init(RelayMAC);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  DebugPrintf("Wakeup cause = %d\n", wakeup_reason);
  DebugPrintf("Boot time = %u; now = %u\n", millis(), GetTimeMS());

  DebugPrintf("Starting BMP280 pressure sensor...\n");
  if (!bmp.begin()) {
      DebugPrint("Could not find a valid BME280 sensor, check wiring, address, sensor ID!\n");
  }
  else{
    DebugPrintf("yay!\n");
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  }

  Wire.begin();

  DebugPrintf("Starting SHT31 temperature/humidity sensor...\n");
  if (! sht31.begin(0x44)) {
    DebugPrint("Couldn't find SHT31\n");
  }

  DebugPrint("Connecting to INA236...\n");
  if (!INA.begin() )
  {
    DebugPrint("could not connect to INA236\n");
  }
  else
  {
    DebugPrint("Connected to INA236!\n");
    INA.setMaxCurrentShunt(2, 0.01);
    //these values stolen from PowerMeterOLED - refinement may be possible here
    //5 would also work without loss of quality
    INA.setBusVoltageConversionTime(6);
    INA.setShuntVoltageConversionTime(6);
    INA.setAverage(16);
    //false -> 80mV, true -> 20mV
    //INA.setADCRange();
    }

  if (wakeup_reason < 1 || wakeup_reason > 5)
  {
    ActualSetup();
  }

  DebugPrintf("Sensors initialized!\n");
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
