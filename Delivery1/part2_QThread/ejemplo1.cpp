#include "ejemplo1.h"

#include <QDebug>

ejemplo1::ejemplo1(): Ui_Counter()
{
	counter = 0;
	timer = new Timer();
	
	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );
	connect(timer, SIGNAL(timeout()), this, SLOT(increase()) );
	
	timer->start(100);
	
}

ejemplo1::~ejemplo1()
{}

void ejemplo1::doButton()
{
	qDebug() << "click on button";
	timer->stop();
}

void ejemplo1::increase()
{
      counter++;
      lcdNumber->display(counter);
}





