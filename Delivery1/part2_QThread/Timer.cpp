#include "Timer.h"

Timer::Timer()
{
  miliseconds = 0;
  isActive = false;
}


void Timer::start(int miliseconds)
{
  this->miliseconds = miliseconds;
  QThread::start();
}

void Timer::run()
{
  isActive = true;
  
  while (true)
  {
    if (isActive)
    {
      msleep(miliseconds);
      emit timeout();
    }
  }
}

void Timer::stop()
{
  isActive = !isActive;
}
