#pragma once

#include <Arduino.h>

//#define ENABLE_DEBUGGING

void DebugInit();
void DebugPrint(String str);
void DebugPrintf(const char* fmt, ...);
void DebugSleep();
