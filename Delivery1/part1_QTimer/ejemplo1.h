#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
Q_OBJECT

private:
  int counter;
  QTimer * timer;
  
public:
    ejemplo1();
    virtual ~ejemplo1();
    
public slots:
	void doButton();
	void increase();
};

#endif // ejemplo1_H
