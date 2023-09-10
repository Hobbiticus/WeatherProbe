#pragma once

//#define SOFTWARE_SERIAL_AVAILABLE

#ifdef SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif


class MHZ19
{
  public:
#ifdef SOFTWARE_SERIAL_AVAILABLE
    MHZ19(int rx, int tx);
#endif
    MHZ19(int pwm);

    int GetCO2();
    bool IsPreheated();
    void ResetPreheatTime();

  private:
#ifdef SOFTWARE_SERIAL_AVAILABLE
    SoftwareSerial m_Serial;
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
    volatile int m_LastPWMReading;
    volatile unsigned long m_LastHighTime;
};
