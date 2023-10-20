#pragma once

#define SOFTWARE_SERIAL_AVAILABLE

#ifdef SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

typedef unsigned long (*GetTimeType)();

class MHZ19
{
  public:
#ifdef SOFTWARE_SERIAL_AVAILABLE
    MHZ19(int rx, int tx, GetTimeType getTimeMS = millis);
#endif
    MHZ19(int pwm, GetTimeType getTimeMS = millis);

    int GetCO2();
    bool IsPreheated();
    void ResetPreheatTime();
    void DoZeroPointCalibration();

  private:
#ifdef SOFTWARE_SERIAL_AVAILABLE
    EspSoftwareSerial::UART m_Serial;
#endif
    bool m_IsHeated;
    unsigned long m_StartTime;

    static unsigned char ComputeChecksum(unsigned char* data)
    {
      unsigned char checksum = 0;
      for (int i = 1; i < 8; i++)
        checksum += data[i];
      checksum = 0xff - checksum;
      checksum++;

      return checksum;
    }
    static void InterruptHigh(void* arg);
    static void InterruptLow(void* arg);
    int m_PWMPin;
    int m_RXPin;
    int m_TXPin;
    volatile int m_LastPWMReading;
    GetTimeType m_GetTimeMS;
    volatile unsigned long m_LastHighTime;
};
