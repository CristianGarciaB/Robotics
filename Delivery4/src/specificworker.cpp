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
SpecificWorker::SpecificWorker ( MapPrx& mprx ) : GenericWorker ( mprx )
{
	
}

/**
 * \brief Default destructor
 */
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams ( RoboCompCommonBehavior::ParameterList params )
{
	try {
		RoboCompCommonBehavior::Parameter par = params.at ( "InnerModelPath" );
		auto innermodel_path = par.value;
		innerModel = new InnerModel ( innermodel_path );
	} catch ( std::exception e ) {
		qFatal ( "Error reading config params" );
	}
	
	
	
	
	timer.start ( Period );
	
	
	return true;
}

void SpecificWorker::gotoTarget()
{
	if ( obstacle() == true ) { // If there is an obstacle ahead, then transit to BUG
		
        state = State::BUG;
		return;
        
	}
	
	float distancia = vectorRobTar.norm2();
	float angulo = atan2 ( vectorRobTar.z(), vectorRobTar.x() );
	
	if ( distancia < 50 ) { //Si ha llegado al objetivo
		state = State::IDLE;
		differentialrobot_proxy->stopBase();
		buffer.setInactive();
        return;
        
	} else {
		float f1, f2, k;
		
		f1 = 0.001 * distancia;
		if ( f1 > 1 )
			f1 = 1;
		
		f2 = exp ( - ( pow ( ( angulo-1.57 ), 2 ) /4 ) );
		
		if ( angulo > 0.57 && angulo < 2.57 )
			k = pow ( ( angulo - 1.57 ), 2 );
		else
			k = 1;
		
		differentialrobot_proxy->setSpeedBase ( f1*f2*advMax, k*rotMax );
		
	}
}


void SpecificWorker::align()
{
	
	float angulo = atan2(vectorRobTar.z(), vectorRobTar.x());
	
	float k;
	
	if(angulo > 0.57 && angulo < 2.57)
		k = pow((angulo - 1.57), 2);
	else
		k = 1;
	
	differentialrobot_proxy->setSpeedBase(0, k*rotMax);
}


bool SpecificWorker::obstacle()
{
	
	bool obs = false;
	
	try {
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data
		
		std::vector<TData>::iterator leftLimit = ldata.begin();
		std::vector<TData>::iterator rightLimit = ldata.begin();
		
		int porcion = ldata.size() / 5;
		//porcion que movemos el iterador para colocarlo en la posición que queremos dentro del rango de valores que recoge el laser
		
		std::advance ( rightLimit, porcion*2 );
		std::advance ( leftLimit, porcion*3 );
		
		//Ordena la parte derecha del array
		std::sort ( ldata.begin(), rightLimit, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
			return a.dist < b.dist;
		} ) ;
		
		//Ordena la parte central del array
		std::sort ( rightLimit, leftLimit, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
			return a.dist < b.dist;
		} ) ;
		
		//Ordena la parte izquierda del array
		std::sort ( leftLimit, ldata.end(), [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
			return a.dist < b.dist;
		} ) ;
		
		float frontDistance = rightLimit->dist;
		float leftDistance = leftLimit->dist;
		float rightDistance = ldata.begin()->dist;
		
		if ( leftDistance < lateralThreshold )
			obs = true;
		else if ( rightDistance < lateralThreshold )
			obs = true;
		else if ( frontDistance < frontThreshold )
			obs = true;
	} catch ( const Ice::Exception &ex ) {
		std::cout << ex << std::endl;
	}
	
	return obs;
}

void SpecificWorker::bug()
{	
	if (targetAtSight())
	{
		state = State::GOTO;
		return;
	}
	align();
	if (!obstacle())
	{
		state = State::GOTO;
		return;
	}
	
	
}

bool SpecificWorker::targetAtSight ()
{
	RoboCompLaser::TLaserData laserData = laser_proxy->getLaserData();
	QPolygonF polygon;
	for ( auto l : laserData ) {
		QVec lr = innerModel->laserTo ( "world", "laser", l.dist, l.angle );
		polygon << QPointF ( lr.x(), lr.z() );
	}
	QVec t = target.getPose();
	return  polygon.containsPoint ( QPointF ( t.x(), t.z() ), Qt::WindingFill );
}

void SpecificWorker::compute()
{
	static RoboCompGenericBase::TBaseState bState;
	
	try {
		differentialrobot_proxy->getBaseState ( bState );
		
		innerModel->updateTransformValues ( "base", bState.x, 0, bState.z ,0, bState.alpha, 0 );
		
		vectorRobTar = innerModel->transform ( "base", QVec::vec3 ( target.x, 0, target.z ),"world" );
		
		switch ( state ) {
			
			case State::IDLE:
				
				if ( buffer.isActive() ) {
					target = buffer.pop();
					state = State::GOTO;
				}
				break;
				
			case State::GOTO:
				gotoTarget();
				break;
				
			case State::BUG:
				bug();
				break;
				
		}
		
	} catch ( const Ice::Exception &e ) {
		std::cout << "Error reading from Camera" << e << std::endl;
	}
}

void SpecificWorker::setPick ( const Pick &myPick )
{
	Target target;
	target.x = myPick.x;
	target.z = myPick.z;
	buffer.push ( target );
}
