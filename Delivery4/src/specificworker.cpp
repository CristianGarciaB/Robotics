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

void SpecificWorker::gotoTarget()
{
	if( obstacle() == true)   // If there is an obstacle ahead, then transit to BUG
	{
		state = State::BUG;
		
		return;
	}
	
	Target t = buffer.pop();
	
	QVec r = innerModel->transform("base", QVec::vec3(t.x, 0, t.z),"world");
	
	float distancia = r.norm2();
	float angulo = atan2(r.z(), r.x());
	
	if (distancia < 50) //Si ha llegado al objetivo
	{
		state = State::IDLE;
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
		
		f2 = exp(-(pow((angulo-1.57), 2)/4));
		
		if(angulo > 0.57 && angulo < 2.57)
			k = pow((angulo - 1.57), 2);
		else
			k = 1;
		
		differentialrobot_proxy->setSpeedBase(f1*f2*advMax, k*rotMax);
		
	}
	
	/*
}
catch(const Ice::Exception &e)
{
std::cout << "Error reading from Camera" << e << std::endl;
}*/
}

/*
 * void SpecificWorker::gotoTarget()
 * {
 *	
 *	if( obstacle() == true)   // If ther is an obstacle ahead, then transit to BUG
 *	{
 *		state = State::BUG;
 *		
 *		return;
 *	}
 *	
 *	//sacar fuera para no pillarlo tantas veces ->en IDLE
 *	//hacer lo de la practica 3 : ir al target
 *	
 *	Target t = buffer.pop();
 *	
 *	QVec rt = innerModel->transform("base", QVec::vec3(t.x, 0, t.z), "world");
 *	
 *	float dist = rt.norm2();
 *	
 *	float ang  = atan2(rt.x(), rt.z());
 *	
 *	
 *	//si ha llegado 
 *	if(dist < 100)          // If close to obstacle stop and transit to IDLE
 *		
 *	{
 *		
 *		state = State::IDLE;
 *		
 *		buffer.setInactive(); //para que IDLE no te vuelva a mandar al mismo sitio
 *		
 *		//y parar el robot
 *		
 *		return;
 *		
 *	}
 *	
 *	float adv = dist;
 *	
 *	if ( fabs( ang ) > 0.05 ) 
 *		adv = 0;
 * }
 */


/**
 * Returns true if there are any obstacle in front of the robot
 *          false in another case.
 * 
 *
 * bool SpecificWorker::obstacle()
 * {
 * 
 *    try
 *    { 
 *        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data
 *        
 *        std::vector<TData>::iterator leftLimit = l.data.begin();
 *        std::vector<TData>::iterator rightLimit = l.data.begin();
 *        
 *        int porcion = l.data.size() / 5;
 *        
 *        std::advance(rightLimit, posicion*2);
 *        std::advance(leftLimit, posicion*3);
 *     
 *        std::
 *        
 *        
 *    }
 *    catch(const Ice::Exception &ex)
 *    {
 *        std::cout << ex << std::endl;
 *    }
 * }
 */

void SpecificWorker::bug()
{
}

bool SpecificWorker::obstacle()
{
	return true;
}

bool SpecificWorker::targetAtSight(RoboCompLaser::TLaserData laserData, Target target)
{
	QPolygonF polygon;
	for (auto l : laserData)
	{
		QVec lr = innerModel->laserTo("world", "laser", l.dist, l.angle);
		polygon << QPointF(lr.x(), lr.z());
	}
	QVec t = target.getPose();
	return  polygon.containsPoint( QPointF(t.x(), t.z() ), Qt::WindingFill );
}

void SpecificWorker::compute()
{
	static RoboCompGenericBase::TBaseState bState;
	
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		
		RoboCompLaser::TLaserData laserData = laser_proxy->getLaserData();
		
		//TODO LO HEMOS CAMBIADO PORQUE SOMOS GUACHIS
		innerModel->updateTransformValues("base", bState.x, 0, bState.z ,0, bState.alpha, 0);        
		
		
		switch( state )
		{
			
			case State::IDLE:
				
				if ( buffer.isActive() )
					state = State::GOTO;
				
				break;
				
			case State::GOTO:
				gotoTarget();
				break;
				
			case State::BUG:
				bug();
				break;
				
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
