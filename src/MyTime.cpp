#include <Arduino.h>

RTC_DATA_ATTR unsigned int CurrentTimeMS = 0;
unsigned long bootTime = millis();
unsigned long GetTimeMS()
{
  return CurrentTimeMS + (millis() - bootTime);
}

void TimeGoToSleep(int howLongMS)
{
    CurrentTimeMS += millis() - bootTime + howLongMS; //in milliseconds
}
