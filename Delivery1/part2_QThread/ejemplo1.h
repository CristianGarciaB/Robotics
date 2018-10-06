#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"
#include "Timer.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
Q_OBJECT

private:
  int counter;
  Timer * timer;
  
public:
    ejemplo1();
    virtual ~ejemplo1();
    
public slots:
	void doButton();
	void increase();
};

#endif // ejemplo1_H
