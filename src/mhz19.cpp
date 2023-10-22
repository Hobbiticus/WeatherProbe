#include <Arduino.h>
#include "mhz19.h"
#include "debug.h"
#include <SoftwareSerial.h>

#define PREHEAT_TIME_SECS 180

#ifdef SOFTWARE_SERIAL_AVAILABLE
MHZ19::MHZ19(int rx, int tx, GetTimeType getTimeMS)
: m_PWMPin(-1), m_RXPin(rx), m_TXPin(tx), m_LastPWMReading(-1), m_GetTimeMS(getTimeMS)
{
  m_Serial.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, rx, tx, false);
}
#endif

MHZ19::MHZ19(int pwm, GetTimeType getTimeMS)
: m_PWMPin(pwm), m_RXPin(-1), m_TXPin(-1), m_LastPWMReading(-2), m_GetTimeMS(getTimeMS)
{
  attachInterruptArg(digitalPinToInterrupt(m_PWMPin), InterruptHigh, this, RISING);
}

void MHZ19::ResetPreheatTime()
{
  m_IsHeated = false;
  m_StartTime = m_GetTimeMS();
}

bool MHZ19::IsPreheated()
{
  m_IsHeated = true;
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
  //DebugPrintf("beep! ^\n");
}

void MHZ19::InterruptLow(void* arg)
{
  MHZ19* mhz = (MHZ19*)arg;
  unsigned long now = micros();
  detachInterrupt(digitalPinToInterrupt(mhz->m_PWMPin));
  attachInterruptArg(digitalPinToInterrupt(mhz->m_PWMPin), InterruptHigh, arg, RISING);
  unsigned long highTimeMS = (now - mhz->m_LastHighTime) / 1000;
  mhz->m_LastPWMReading = (highTimeMS - 2) * 5;
  //DebugPrintf("boop! v %d\n", mhz->m_LastPWMReading);
}

int MHZ19::GetCO2()
{
  if (!IsPreheated())
  {
    DebugPrintf("Not heated yet, but got %d\n", m_LastPWMReading);
    return -1;
  }

  if (m_PWMPin >= 0)
    return m_LastPWMReading;

#ifdef SOFTWARE_SERIAL_AVAILABLE
  unsigned char request[9];
  memset(request, 0, sizeof(request));
  request[0] = 0xFF;
  request[1] = 0x01;
  request[2] = 0x86;
  request[8] = ComputeChecksum(request);

  int wrote = m_Serial.write(request, sizeof(request));
  if (wrote != sizeof(request))
  {
    DebugPrintf("Wrote %d to serial\n", wrote);
    return -4;
  }

  unsigned char response[9];
do_read:
  int startTime = millis();
  while (m_Serial.available() < sizeof(response))
  {
    if (millis() - startTime > 2000)
    {
      DebugPrintf("available = %d\n", m_Serial.available());
      return -2;
    }
    delay(10);
  }
  DebugPrintf("available = %d\n", m_Serial.available());
  m_Serial.readBytes(response, 1);
  if (response[0] != 0xFF)
    goto do_read;
  m_Serial.readBytes(response+1, 1);
  if (response[1] != 0x86)
    goto do_read;
  m_Serial.readBytes(response+2, sizeof(response)-2);
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

void MHZ19::DoZeroPointCalibration()
{
#ifdef SOFTWARE_SERIAL_AVAILABLE
  unsigned char msg[9] = {0};
  msg[0] = 0xFF;
  msg[1] = 0x01;
  msg[2] = 0x87;
  msg[8] = ComputeChecksum(msg);
  m_Serial.write(msg, 9);
#endif
}
