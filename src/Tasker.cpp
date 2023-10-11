#include <Arduino.h>
#include "Tasker.h"
#include "MyTime.h"


void TaskInit(Task& task, unsigned long readingPeriod, unsigned long warmupTime)
{
    task.m_ReadingPeriod = readingPeriod;
    task.m_WarmupTime = warmupTime;
    task.m_State = Task::WARMING;
    task.m_LastReadingTime = 0;
    task.m_WarmupStartTime = millis();
}

unsigned long TaskGetNextEventTime(Task& task)
{
    switch (task.m_State)
    {
    case Task::SLEEPING:
        return task.m_LastReadingTime + task.m_ReadingPeriod - task.m_WarmupTime;
    case Task::WARMING:
        return task.m_WarmupStartTime + task.m_WarmupTime;
    }
    return (unsigned long)-1;
}

int TaskTick(Task& task)
{
    unsigned long now = GetTimeMS();
    unsigned long nextEventTime = TaskGetNextEventTime(task);
    if (nextEventTime - now < 0x7FFFFFFF)
        return 0; //nothing to do yet

    //move to the next state
    switch (task.m_State)
    {
        case Task::SLEEPING:
            //no warmup necessary - just do a reading
            if (task.m_WarmupTime == 0)
            {
                task.m_LastReadingTime += task.m_ReadingPeriod;
                return 2;
            }
            task.m_State = Task::WARMING;
            task.m_WarmupStartTime = now;
            return 1;
        case Task::WARMING:
            task.m_State = Task::SLEEPING;
            task.m_LastReadingTime += task.m_ReadingPeriod;
            return 2;
    }
    return 0;
}
