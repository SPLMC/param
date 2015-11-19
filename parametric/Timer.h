#ifndef __TIMER_H__
#define __TIMER_H__

namespace parametric {

class Timer
{
 private:
  double started;
  double time;

 public:
  Timer() { started = -1; time = 0; }
  Timer(const Timer &rhs);  
  const Timer &operator = (const Timer &rhs);

  bool running() const;
  void start();
  void stop();
  void forward(const Timer &arg);
  double read() const;
};

}

#endif
