#ifndef Timer_H
#define Timer_H

#include <QThread>

class Timer : public QThread
{
Q_OBJECT

private:
  int miliseconds;
  bool isActive;

public:
  Timer();
  void start(int miliseconds);
  void run();
  void stop();
  
signals:
  void timeout();
  
};


#endif 