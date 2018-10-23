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

    if( obstacle == true)   // If ther is an obstacle ahead, then transit to BUG

   {

      state = State::BUG;

      return;

   }

    Target t = buffer.pop();
   
    QVec rt = innerModel->transform("base", QVec::vec3(t.x, 0, t.z), "world");

    float dist = rt.norm2();

    float ang  = atan2(rt.x(), rt.z());

   if(dist < 100)          // If close to obstacle stop and transit to IDLE

  {

    state = State::IDLE;

    buffer.setActive();

   return;

  }

  float adv = dist;

  if ( fabs( rot) > 0.05 )

   adv = 0;

 }
// void SpecificWorker::bug()
// 
// {
// 
// }
// 
// bool SpecificWorker::obstacle()
// 
// {
// 
// }
// 
// bool SpecificWorker::targetAtSight()
// 
// {
// 
// }


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
