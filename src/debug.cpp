#include "debug.h"
#include "MyTime.h"

#ifdef ENABLE_DEBUGGING

#include "EspNowRelay.h"
#include <stdarg.h>

//socket debugging
//#define DEBUG_ADDRESS "192.168.1.55"
#define DEBUG_ADDRESS ((192 << 0) | (168 << 8) | (1 << 16) | (55 << 24))
#define DEBUG_PORT 4565
unsigned char RelayMAC[6] = {0xC8, 0xC9, 0xA3, 0xD2, 0x9D, 0xC8};

int DebugSocket = 0;
static EspNowRelay Relay;

#endif

void DebugInit()
{
#ifdef ENABLE_DEBUGGING
    Relay.Init(RelayMAC);
    DebugSocket = Relay.Connect(DEBUG_ADDRESS, DEBUG_PORT);
#endif
}

void DebugPrint(String str)
{
  Serial.print(str);
#ifdef ENABLE_DEBUGGING
  //reconnect if we're not connected
  if (!DebugSocket)
  {
    DebugSocket = Relay.Connect(DEBUG_ADDRESS, DEBUG_PORT);
    if (!DebugSocket)
      return; //failed to reconnect - give up
  }

  if (Relay.Send(DebugSocket, (const unsigned char*)str.c_str(), str.length()) == -1)
  {
    Relay.Close(DebugSocket);
    //try again
    DebugSocket = Relay.Connect(DEBUG_ADDRESS, DEBUG_PORT);
    if (!DebugSocket)
        return;
    if (Relay.Send(DebugSocket, (const unsigned char*)str.c_str(), str.length()) == -1)
    {
        //ahh well, we tried
        Relay.Close(DebugSocket);
        DebugSocket = 0;
    }
  }
#endif
}

void DebugPrintf(const char* format, ...)
{
    va_list arg;
    va_start(arg, format);
    char temp[64];
    char* buffer = temp;
    size_t len = vsnprintf(temp, sizeof(temp), format, arg);
    va_end(arg);
    if (len > sizeof(temp) - 1) {
        buffer = new char[len + 1];
        if (!buffer) {
            return;
        }
        va_start(arg, format);
        vsnprintf(buffer, len + 1, format, arg);
        va_end(arg);
    }

    char timestamp[12] = {0};
    snprintf(timestamp, sizeof(timestamp), "%8u: ", (unsigned int)GetTimeMS());
    DebugPrint(timestamp);

    DebugPrint(buffer);

    if (buffer != temp) {
        delete[] buffer;
    }
    return;
}

void DebugSleep()
{
#ifdef ENABLE_DEBUGGING
    if (DebugSocket)
    {
        Relay.Close(DebugSocket);
        DebugSocket = 0;
    }
#endif
}
