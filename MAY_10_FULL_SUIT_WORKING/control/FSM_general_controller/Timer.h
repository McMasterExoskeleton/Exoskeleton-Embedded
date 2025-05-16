#ifndef TIMER_H
#include <chrono>

class Timer
{

private:
  static Timer *sInstance;

  std::chrono::system_clock::time_point mStartTime; // start time after resetting timer
  std::chrono::duration<float> mDeltaTimer;
  float mTimeScale;

  Timer();
  ~Timer();

public:
  static Timer *instance(); // retrives instance
  static void Release();

  void Reset();
  float DeltaTime();

  void TimeScale(float t = 1.0f);

  float TimeScale();

  void Tick();
};

#endif