#pragma once


struct Task
{
    enum State
    {
        SLEEPING = 0,
        WARMING,
    };

    unsigned long m_ReadingPeriod;
    unsigned long m_WarmupTime;

    unsigned long m_LastReadingTime;
    unsigned long m_WarmupStartTime;
    State m_State;
};

void TaskInit(Task& task, unsigned long readingPeriod, unsigned long warmupTime);
unsigned long TaskGetNextEventTime(Task& task);
//returns 0 when nothing to be done
//        1 when transitioning to warmup
//        2 when ready for reading
int TaskTick(Task& task);
