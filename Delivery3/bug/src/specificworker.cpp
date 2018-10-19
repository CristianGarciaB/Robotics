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
		innerModel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
		
		if(buffer.isActive())
		{
			Target t = buffer.pop();
			//Version de Pablo, comparar con la nuestra
			QVec r = innerModel->transform("robot", QVec::vec3(t.x, 0, t.z),"world");
			
			float distancia = r.norm2();
			if (distancia < 50) //Si ha llegado al objetivo
			{
				differentialrobot_proxy->stopBase();
				buffer.setInactive();
			}
			else
			{
				float rot = -atan2(r.z(), r.x());
				float adv = distancia;
				float f1, f2, k;
				
				if (distancia < 1000)
				{
					f1 = 1/1000 * distancia;
				}
				else
				{
					f1 = 1;
				}
				
				
				//differentialrobot_proxy->setSpeedBase(f1*f2*adv, k*rot);
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
	//subscribesToCODE
	//Hacer push a la estructura
	
	std::cout << myPick.x << " " << myPick.z << std::endl;
}


