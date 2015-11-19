#include <cassert>
#include <sys/time.h>
#include "Timer.h"

namespace parametric {
  using namespace std;

/*********************************************************************/
//constructors and destructors
/*********************************************************************/
Timer::Timer(const Timer &rhs) { *this = rhs; }

/*********************************************************************/
//operators
/*********************************************************************/
const Timer &Timer::operator = (const Timer &rhs)
{
  started = rhs.started;
  time = rhs.time;
  return *this;
}

/*********************************************************************/
//return true if the timer is running and false otherwise
/*********************************************************************/
bool Timer::running() const {
  return (started != -1);
}

/*********************************************************************/
//read the current time. the timer must be stopped.
/*********************************************************************/
double Timer::read() const {
  if (started != -1) {
    assert(false);
  }
  return time;
}

/*********************************************************************/
//forward a timer by the supplied amount of time
/*********************************************************************/
void Timer::forward(const Timer &arg) {
  if (arg.started != -1) {
    assert(false);
  }
  if (started != -1) {
    assert(false);
  }
  time += arg.time;
}

/*********************************************************************/
//start the timer
/*********************************************************************/
void Timer::start() {
  if (started != -1) {
    assert(false);
  }
#ifdef WIN32
  started = static_cast<double>(::time(NULL));
#else
  struct timeval tv;
  if (gettimeofday(&tv, 0) == -1) {
    assert(false);
  }
  started = tv.tv_sec + (tv.tv_usec / 1000000.0);
#endif //WIN32
}

/*********************************************************************/
//stop the timer
/*********************************************************************/
void Timer::stop() {  
  if(started == -1) {
    assert(false);
    return;  
  }
#ifdef WIN32
  double currTime = static_cast<double>(::time(NULL));
#else
  struct timeval tv;
  if (gettimeofday(&tv, 0) == -1) {
    assert(false);
  }
  double currTime = tv.tv_sec + (tv.tv_usec / 1000000.0);
#endif //WIN32
  time += (currTime - started);
  started = -1;
}

} //end of util

/*********************************************************************/
//end of Timer.cpp
/*********************************************************************/
