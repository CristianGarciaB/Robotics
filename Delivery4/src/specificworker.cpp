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

//CONSTRUCTOR AND DESTRUCTOR
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
//--------------------------


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

void SpecificWorker::setPick ( const Pick &myPick )
{
	Pose target;
	target.x = myPick.x;
	target.z = myPick.z;
	buffer.push ( target );
}


//-----------AUX METHODS-----------
/**
* Transform max and min angle from laser system to robot system
*/
bool SpecificWorker::viaLibre()
{
	bool targetEnRango = false;

	//Transforma el ángulo max y min del sist. del láser al sist. del robot 
	//(Ver documentacion)
	float angleMaxSisRobot = (angleMax - (M_PI/2)) * -1; 
	float angleMinSisRobot = (angleMin - (M_PI/2)) * -1;
	
	if (angleMinSisRobot < angle && angle < angleMaxSisRobot)
		targetEnRango = true;
	
	Section section;
	
	if (angle >= M_PI/2) //Si el ángulo del target es mayor que 90º, está en la parte izq, sino en la der
		section = Section::LEFT;
	else
		section = Section::RIGHT;
	
	if (targetEnRango && !obstacle(section, partObstacle) && !obstacle(Section::FRONT, partObstacle)) //target en rango y no hay obs en total
		std::cout<<"VIA LIBRE"<<std::endl;
    
	return targetEnRango;
}

bool SpecificWorker::obstacle(Section section, int &partObstacle)
{
	bool obs = false;
	std::vector<TData>::iterator init;
	std::vector<TData>::iterator fin;
	float frontDistance, leftDistance, rightDistance;
	
	switch (section){
        
        case Section::FRONT:{
            ordenarLaser(ldata);
            frontDistance = minFront->dist;
            if (frontDistance < 350){
                
                std::cout<<"---------------------- SECTION FRONT -------------------"<<std::endl;
                obs = true;
                partObstacle=0;
            }
        }break;
		
		case Section::LEFT:{
			init = ldata.begin();
			fin = ldata.end();
			std::advance(init, ldata.size()/2);
			
			std::sort ( init, fin, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
				return a.dist < b.dist;
			} ) ;
			
			if (init->dist < 300){
        
                std::cout<<"---------------------- SECTION LEFT -------------------"<<std::endl;
				obs = true;
                partObstacle=-1;
            }
        }break;
			
		case Section::RIGHT:{
			init = ldata.begin();
			fin = ldata.begin();
			std::advance(fin, ldata.size()/2);
			
			std::sort ( init, fin, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
				return a.dist < b.dist;
			} ) ;
			
			if (init->dist < 300){
                
                std::cout<<"---------------------- SECTION RIGHT -------------------"<<std::endl;
				obs = true;
                partObstacle=1;
                
            }
        }break;
			
		case Section::TOTAL:{
			ordenarLaser(ldata);
			frontDistance = minFront->dist;
            leftDistance = minLeft->dist;
			rightDistance = minRight->dist;
			
			if ( leftDistance < 300){
                
                std::cout<<"---------------------- SECTION TOTAL:LEFT -------------------"<<std::endl;
				obs = true;
                partObstacle=-1;
				
			}else if ( rightDistance < 300){
                
                std::cout<<"---------------------- SECTION TOTAL:RIGHT -------------------"<<std::endl;
				obs = true;
                partObstacle=1;
				
			}else if ( frontDistance < 350){
                std::cout<<"---------------------- SECTION TOTAL:FRONT -------------------"<<std::endl;
				obs = true;
                partObstacle=0;
			}
		}break;
        
        case Section::middleLEFT:{
            
            ordenarLaser(ldata);
            leftDistance = middleLeft->dist;
            if ( leftDistance < 200){
                
                std::cout<<"---------------------- SECTION MID LEFT -------------------"<<std::endl;
                obs=true;
                partObstacle = -2;
            }
        }break;
        
        case Section::middleRIGHT:{
            
            ordenarLaser(ldata);
            rightDistance = middleRight->dist;
            if ( rightDistance < 200){
                
                std::cout<<"---------------------- SECTION MID RIGHT -------------------"<<std::endl;
                obs=true;
                partObstacle = 2;
            }
            
        }break;
        
	}
	
	
	return obs;
}

void SpecificWorker::ordenarLaser(RoboCompLaser::TLaserData ldata)
{
	
    minRight = ldata.begin(); //Minimum distance at right
    minLeft = ldata.begin(); //Minimum distance at left
	minFront = ldata.begin(); //Minimum distance in front
    middleLeft = ldata.begin(); //Minimum distance at left
	middleRight = ldata.begin(); //Minimum distance in front
	
	int portion = ldata.size() / 7;
	//porcion que movemos el iterador para colocarlo en la posición que queremos dentro del rango de valores que recoge el laser
	
	std::advance ( minFront, portion*3 );
	std::advance ( minLeft, portion*4 );
    
    std::advance ( middleLeft, portion*6 );
    std::advance ( middleRight, portion*1 );
    
	//Sorts rigth part of the data laser vector and returns first minimum distance to any obstacle
	std::sort ( minRight, middleRight, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
		return a.dist < b.dist;
	} ) ;
    
    std::sort ( middleRight, minFront, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
		return a.dist < b.dist;
	} ) ;
	
	//Sorts front part of the data and returns first minimum distance to any obstacle
	std::sort ( minFront, middleLeft, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
		return a.dist < b.dist;
	} ) ;
	
	//Sorts left part of the data and returns first minimum distance to any obstacle
	std::sort ( middleLeft, ldata.end(), [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ) {
		return a.dist < b.dist;
	} ) ;
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

//---------------------------------



//-------------METHODS-------------

void SpecificWorker::align()
{
    std::cout<<"-------------------------------------------ALINEANDO----------------------------------"<<std::endl;
	float angle = atan2(vectorRobTar.z(), vectorRobTar.x());
	
    if (angle < 1.7 && angle > 1.40) //Si ya está alineado
	{
        std::cout << "Estoy alineado" << std::endl;
        state = State::GOTO;
		return;
	}
	
	float k =1;
    
    if (abs(angle) > 1.57)
        k = -1;
	
	differentialrobot_proxy->setSpeedBase(0, k*rotMax);
}

void SpecificWorker::gotoTarget()
{

    distance = vectorRobTar.norm2();
    //si hay obstaculo en el frente, derecha o izquierda 
	if (( obstacle(Section::FRONT, partObstacle) ||
         obstacle(Section::RIGHT, partObstacle) ||
         obstacle(Section::LEFT, partObstacle) ) && distance > 300 ) { // If there is an obstacle ahead, then transit to BUG
		std::cout << "There are an obstacle, go to STATE::BUG" << std::endl;
		state = State::BUG;
		return;
	}

	//si hay obstaculo pero la distancia es menor que 100 el target esta cerca de pared
	if( obstacle(Section::FRONT, partObstacle) && distance<200){
        std::cout <<"ON TARGET. STOP AND WAIT NEW TARGET" << std::endl;
		state = State::IDLE;
		differentialrobot_proxy->stopBase();
		buffer.setInactive();
		return;
    }
	
    //si no hay obstaculo y la distancia es menor que 100, llegado al target
	if ( distance < 200) { 
		std::cout <<"ON TARGET. STOP AND WAIT NEW TARGET" << std::endl;
		state = State::IDLE;
		differentialrobot_proxy->stopBase();
		buffer.setInactive();
		return;
	} 
	//si no hay obstaculo en el frente pero puede haber O NO en la derecha o la izquierda
	if ( !obstacle(Section::FRONT, partObstacle) &&
         (obstacle(Section::RIGHT, partObstacle) ||
          obstacle(Section::LEFT, partObstacle)  ||
          !obstacle(Section::RIGHT, partObstacle)||
          !obstacle(Section::LEFT, partObstacle)    )   ){
		float f1, f2, k;
		
		float distanceObs = minFront->dist;
		float dist;
		
		if(distanceObs < distance)
			dist = distanceObs;
		else
			dist = distance;
		
		f1 = 0.001 * dist;
		if ( f1 > 1 )
			f1 = 1;
		
		f2 = exp ( - ( pow ( ( angle-1.57 ), 2 ) /4 ) );
		
		if ( angle > 0.57 && angle < 2.57 )
			k = pow ( ( angle - 1.57 ), 2 );
		else
			k = 1;
		
		differentialrobot_proxy->setSpeedBase ( f1*f2*advMax, k*rotMax );
		
	}
}

void SpecificWorker::bug()
{
    partObstacle = 9999; //initialized with an invalid value
     
	//si hay obstaculo en el frente, derecha o izquierda
	if (obstacle(Section::FRONT, partObstacle) ||
        obstacle(Section::RIGHT, partObstacle) ||
        obstacle(Section::LEFT, partObstacle) )
	{
		state = State::PARALLEL;
		return;
	}
	
	
	//si hay obstaculo en la derecha o la izquierda
	if (obstacle(Section::middleLEFT, partObstacle) || obstacle(Section::middleRIGHT, partObstacle) ||
        !obstacle(Section::middleLEFT, partObstacle) || !obstacle(Section::middleRIGHT, partObstacle)){
        
        differentialrobot_proxy->setSpeedBase(advMax/1.5, 0);
        if (targetAtSight()){
            state = State::ALIGN;
            return;
        }
        
        
    }
    
    std::cout<<partObstacle<<std::endl;
	
	float dist = std::prev(ldataRaw.end())->dist;
	float velRot = 0;
    
	if (dist < (lateralThreshold - 50))
		velRot = 0.5;
    
	if (dist > (lateralThreshold + 50))
		velRot = -0.5;
    
	differentialrobot_proxy->setSpeedBase ( 250, velRot );
}

void SpecificWorker::parallel()
{
    std::cout<<partObstacle<<std::endl;
	float velRot = 0;
    


    if (partObstacle == -1){ //The obstacle is on the left, robot turns to right
        velRot = 0.15;
   
        differentialrobot_proxy->setSpeedBase ( 0, rotMax*velRot);
        state = State::BUG;
        return;

    }
    
    if (partObstacle == 1){ //The obstacle is on the right, robot turns to left
        velRot = -0.15;

        differentialrobot_proxy->setSpeedBase ( 0, rotMax*velRot);
        state = State::BUG;
        return;
    }
    
    if (partObstacle == 0){ //The obstacle is in front, robot turns to right
        
        velRot = 2;
        differentialrobot_proxy->setSpeedBase ( 0, rotMax*velRot);
        state = State::BUG;
        return;
    } 
    
        if (partObstacle == -2){ //The obstacle is on the left, robot turns to right
        velRot = 0.05;
   
        differentialrobot_proxy->setSpeedBase ( 0, rotMax*velRot);
        state = State::BUG;
        return;

    }
    
    if (partObstacle == 2){ //The obstacle is on the right, robot turns to left
        velRot = -0.05;

        differentialrobot_proxy->setSpeedBase ( 0, rotMax*velRot);
        state = State::BUG;
        return;
    }

}

//---------------------------------


void SpecificWorker::compute()
{
	
	std::cout <<"--------------------------------------"<< std::endl;
	static RoboCompGenericBase::TBaseState bState;
	
	try {
		differentialrobot_proxy->getBaseState ( bState );
		
		innerModel->updateTransformValues ( "base", bState.x, 0, bState.z ,0, bState.alpha, 0 );
		
		vectorRobTar = innerModel->transform ( "base", QVec::vec3 ( target.x, 0, target.z ),"world" );
		distance = vectorRobTar.norm2();
		angle = atan2 ( vectorRobTar.z(), vectorRobTar.x() );
		
		ldata = laser_proxy->getLaserData();  //read laser data
		ldataRaw = laser_proxy->getLaserData();
		angleMax = std::prev(ldata.end())->angle;
		angleMin = ldata.begin()->angle;
		
		ordenarLaser(ldata);
		
		if (buffer.isActive()){
			state = State::IDLE;
		}
		
		switch ( state ) {
			
            case State::IDLE:
                std::cout << "Waiting to obtain a target" << std::endl;
                differentialrobot_proxy->stopBase();
                if ( buffer.isActive() ) {
                    target = buffer.pop();
                    buffer.setInactive();
                    state = State::ALIGN;
                }
                break;
                    
            case State::ALIGN:
                std::cout << "Align to target" << std::endl;
                align();
                break;
				
			case State::GOTO:
				std::cout << "Go to target" << std::endl;
				gotoTarget();
				break;
				
			case State::BUG:
				std::cout << "Found an obstacle" << std::endl;
				bug();
				break;

			case State::PARALLEL:
				std::cout << "Parallel to the obstacle" << std::endl;
				parallel();
				break;
		}
		
	} catch ( const Ice::Exception &e ) {
		std::cout << "Error reading from Camera" << e << std::endl;
	}
}
