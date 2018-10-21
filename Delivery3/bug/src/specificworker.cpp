/*
 *    Copyright (C)2018 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <math.h>

/**
 * \brief Default constructor
 */
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	
}

/**
 * \brief Default destructor
 */
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		auto innermodel_path = par.value;
		innerModel = new InnerModel(innermodel_path);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }
	
	
	
	
	timer.start(Period);
	
	
	return true;
}

void SpecificWorker::compute()
{
	static RoboCompGenericBase::TBaseState bState;
	
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		
		if(buffer.isActive())
		{
			Target t = buffer.pop();
			
			//Version de Pablo
			QVec r = innerModel->transform("base", QVec::vec3(t.x, 0, t.z),"world");
			qDebug() <<"Pablo: " << r;
			//Nuestra versión
			Rot2D ro (bState.alpha);
			QVec y = QVec :: vec2 (t.x, t.z);
			QVec T = QVec :: vec2 (bState.x, bState.z);
			QVec x = ro.invert() * (y - T);
			qDebug() <<"Nuestro: " << x;
			
			float distancia = r.norm2();
			float angulo = atan2(r.z(), r.x());
			
			if (distancia < 50) //Si ha llegado al objetivo
			{
				differentialrobot_proxy->stopBase();
				buffer.setInactive();
			}
			else
			{
				float rotMax = 2;
				float advMax = 1000;
				float f1, f2, k;
				
				f1 = 0.001 * distancia;
				if (f1 > 1)
					f1 = 1;
				
				f2 = exp(-pow((angulo-1.57), 2));
				
				if(angulo > 0.57 && angulo < 2.57)
					k = pow((angulo - 1.57), 2);
				else
					k = 1;
				
				differentialrobot_proxy->setSpeedBase(f1*f2*advMax, k*rotMax);
				
			}
		}
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Camera" << e << std::endl;
	}
}


void SpecificWorker::setPick(const Pick &myPick)
{
	Target target;
	target.x = myPick.x;
	target.z = myPick.z;
	buffer.push(target);
}


