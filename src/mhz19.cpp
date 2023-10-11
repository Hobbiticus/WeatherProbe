#include <Arduino.h>
#include "mhz19.h"

#define PREHEAT_TIME_SECS 180

#ifdef SOFTWARE_SERIAL_AVAILABLE
MHZ19::MHZ19(int rx, int tx)
: m_PWMPin(-1), m_LastPWMReading(-1)
{
  m_Serial.begin(9600, SWSERIAL_8N1, 14, 21, false);
}
#endif

MHZ19::MHZ19(int pwm, GetTimeType getTimeMS)
: m_PWMPin(pwm), m_LastPWMReading(-1), m_GetTimeMS(getTimeMS)
{
  attachInterruptArg(digitalPinToInterrupt(m_PWMPin), InterruptHigh, this, RISING);
}

void MHZ19::ResetPreheatTime()
{
  m_IsHeated = false;
  m_StartTime = m_GetTimeMS();
}

void MHZ19::SetPreheatStartTime(unsigned long whenMS)
{
  m_StartTime = whenMS;
  IsPreheated();
  Serial.printf("%d ms until CO2 heated\n",(int)(1000 * PREHEAT_TIME_SECS - (m_GetTimeMS() - m_StartTime)));
}

bool MHZ19::IsPreheated()
{
  if (m_IsHeated)
    return true;
  m_IsHeated = m_GetTimeMS() - m_StartTime > 1000 * PREHEAT_TIME_SECS;
  return m_IsHeated;
}

void MHZ19::InterruptHigh(void* arg)
{
  MHZ19* mhz = (MHZ19*)arg;
  mhz->m_LastHighTime = micros();
  detachInterrupt(digitalPinToInterrupt(mhz->m_PWMPin));
  attachInterruptArg(digitalPinToInterrupt(mhz->m_PWMPin), MHZ19::InterruptLow, arg, FALLING);
}

void MHZ19::InterruptLow(void* arg)
{
  MHZ19* mhz = (MHZ19*)arg;
  unsigned long now = micros();
  detachInterrupt(digitalPinToInterrupt(mhz->m_PWMPin));
  attachInterruptArg(digitalPinToInterrupt(mhz->m_PWMPin), InterruptHigh, arg, RISING);
  unsigned long highTimeMS = (now - mhz->m_LastHighTime) / 1000;
  mhz->m_LastPWMReading = (highTimeMS - 2) * 5;
}

int MHZ19::GetCO2()
{
  if (!IsPreheated())
    return -1;

  if (m_PWMPin >= 0)
    return m_LastPWMReading;

#ifdef SOFTWARE_SERIAL_AVAILABLE          
  unsigned char request[9];
  memset(request, 0, sizeof(request));
  request[0] = 0xFF;
  request[1] = 0x01;
  request[2] = 0x86;
  request[8] = ComputeChecksum(request);

  m_Serial.write(request, sizeof(request));

  unsigned char response[9];
  int startTime = millis();
  while (m_Serial.available() != sizeof(response))
  {
    if (millis() - startTime > 100)
      return -2;
    delay(10);
  }
  m_Serial.readBytes(response, sizeof(response));
  unsigned char responseChecksum = ComputeChecksum(response);
  if (responseChecksum != response[8])
  {
    Serial.println("Response checksum failed");
    return -3;
  }

  int ret = response[2];
  ret <<= 8;
  ret += response[3];
  return ret;
#else
  return -1;
#endif
}