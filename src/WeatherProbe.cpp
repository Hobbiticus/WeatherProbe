#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PMserial.h> // Arduino library for PM sensors with serial interface
#include "debug.h"
#include "MyTime.h"
#include "Tasker.h"
#include "EspNowRelay.h"
#include "../../CentralBrain/include/IngestProtocol.h"
#include "../../CentralBrain/include/WeatherProtocol.h"

unsigned char RelayMAC[6] = {0xC8, 0xC9, 0xA3, 0xD2, 0x9D, 0xC8};
unsigned char BrainAddr[4] = {192, 168, 1, 222};

const unsigned short IngestPort = 7777;

//3 to 2 yields range 0-8.25V, battery should get to max 7.3
float R1 = 300.0;
float R2 = 200.0;
float VIN = 3.3;

//actual voltage, measured voltage
const int VoltageCalibrationTableSize = 5;
float VoltageCalibrationTable[VoltageCalibrationTableSize][2] =
{
  { 5.5, 5.06 },
  { 6.0, 5.57 },
  { 6.5, 6.10 },
  { 7.0, 6.76 },
  { 7.5, 7.58 }
};


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
  TASK_BATT_LEVEL,
  //TASK_CO2,
  TASK_PM,
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
    DebugPrintf("sensor id: %u\n", bme.sensorID());
    if (bme.sensorID() == 0)
      return false;
    float temp = bme.readTemperature();
    float humid = bme.readHumidity();
    float pressure = bme.readPressure();
    DebugPrintf(" ========= temp = %.2f, humid = %.2f, pressure = %.2f\n", temp, humid, pressure);
    //sometimes temperature and pressure return anomolous readings (this might be covered with the sensorID check above but this should guarantee it)
    if (temp == 0 && pressure < 100000.0)
      return false;
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
    //sometimes this reads as 0 when there is definitely voltage here
    const int NumReadings = 5;
    //String out;

    int voltages[NumReadings] = {0};

    int readingCount = 0;
    const int MaxReadings = NumReadings * 20;

    for (int i = 0; i < NumReadings && readingCount < MaxReadings; readingCount++)
    {
      int voltage = analogRead(BATT_LEVEL_PIN);
      //DebugPrint("Reading = " + String(voltage) + "\n");
      if (voltage != 0)
      {
        //out += " " + String(voltage);
        voltages[i] = voltage;
        i++;
      }
      if (i < NumReadings - 1)
        //really need to wait a while for this to be accurate for some reason
        delay(100);
    }
    //DebugPrint(out + "\n");

    //sort the voltages (go go gadget bubble sort!)
    bool sorted = false;
    while (!sorted)
    {
      sorted = true;
      for (int i = 0; i < NumReadings-1; i++)
      {
        if (voltages[i] > voltages[i+1])
        {
          sorted = false;
          int temp = voltages[i];
          voltages[i] = voltages[i+1];
          voltages[i+1] = temp;
          break;
        }
      }
    }

    //drop the top and bottom values and average
    float rawVoltage = 0;
    for (int i = 1; i < NumReadings-1; i++)
      rawVoltage += voltages[i];
    rawVoltage /= NumReadings - 2;

    Serial.print("Raw reading = ");
    Serial.println(rawVoltage);

    float vout = (rawVoltage * VIN) / 4096.0;
    float vin = vout / (R2 / (R1 + R2));
    DebugPrint("Meastured voltage = " + String(vin, 2) + String(" V\n"));

    //use the calibration table to get a more accurate voltage reading
    if (vin < VoltageCalibrationTable[0][1])
    {
      //extrapolate down
      float ratio = VoltageCalibrationTable[0][0] / VoltageCalibrationTable[0][1];
      vin *= ratio;
    }
    else if (vin > VoltageCalibrationTable[VoltageCalibrationTableSize-1][1])
    {
      //extrapolate up
      float ratio = VoltageCalibrationTable[VoltageCalibrationTableSize-1][0] / VoltageCalibrationTable[VoltageCalibrationTableSize-1][1];
      vin *= ratio;
    }
    else
    {
      //interpolate!
      int i = 0;
      //find where to interpolate
      for (; i < VoltageCalibrationTableSize - 1; i++)
      {
        if (VoltageCalibrationTable[i+1][1] > vin)
          break;
      }
      DebugPrint("Interpolating between " + String(VoltageCalibrationTable[i][1], 2) + " and " + String(VoltageCalibrationTable[i+1][1], 2) + "\n");
      float ratio = (vin - VoltageCalibrationTable[i][1]) / (VoltageCalibrationTable[i+1][1] - VoltageCalibrationTable[i][1]);
      vin = (VoltageCalibrationTable[i+1][0] * ratio) + (VoltageCalibrationTable[i][0] * (1 - ratio));
    }
    DebugPrint("Actual voltage = " + String(vin, 2) + String(" V\n"));

    DebugPrintf("BATTERY READING = %d -> %.2f Volts\n", (int)rawVoltage, vin);
    data.m_Voltage = (unsigned int)(vin * 100);
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

  //setup pins
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
