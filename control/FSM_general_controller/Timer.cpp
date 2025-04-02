#include "Timer.h"

Timer *Timer::sInstance = nullptr;

Timer *Timer::instance() // create new timer
{
  if (sInstance == nullptr)
  {
    sInstance = new Timer();
  }
  return sInstance;
}

void Timer::Release() // delete timer
{
  delete sInstance;
  sInstance = nullptr;
}

Timer::Timer()
{
  Reset();
  mTimeScale = 1.0f;
  mDeltaTimer = std::chrono::duration<float>(0.0f);
}

Timer::~Timer()
{
}

void Timer::Reset()
{
  mStartTime = std::chrono::system_clock::now(); // current time
}

float Timer::DeltaTime()
{
  return mDeltaTimer.count();
}

void Timer::TimeScale(float t)
{
  mTimeScale = t;
}

float Timer::TimeScale()
{
  return mTimeScale;
}

void Timer::Tick()
{
  mDeltaTimer = std::chrono::system_clock::now() - mStartTime;
}
