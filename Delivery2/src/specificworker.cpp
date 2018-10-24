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
//       THE FOLLOWING IS JUST AN EXAMPLE
//
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		innermodel_path = par.value;
//		innermodel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }




	timer.start(Period);


	return true;
}


void SpecificWorker::compute()
{
	const float FrontThreshold = 335; //millimeters
	const float LateralThreshold = 235; //millimeters
    float rot = 0.9;  //Velocidad de rotacion rads per second
    int velAvanceGiro = 5; //Velocidad de avance durante el giro

    try
    { 
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data
        
        std::vector<TData>::iterator medio = ldata.begin();
        std::vector<TData>::iterator medio2 = ldata.begin();
        int posicion = ldata.size() / 5;
        
        std::advance(medio, posicion*2);
        std::advance(medio2, posicion*3);
        
        //Ordena la parte derecha del array
        std::sort( ldata.begin(), medio, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ; 
        
        //Ordena la parte central del array
       std::sort( medio,   medio2, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ; 
        
        //Ordena la parte izquierda del array
        std::sort( medio2, ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ; 
        
        float FrontAngle = medio->angle;
        float FrontDistance = medio->dist;
        float IzqDistance = medio2->dist;
        float DerDistance =ldata.front().dist;
        int tiempoDormido = rand()%(1500000-100000 + 1) + 100000;
        
        if(IzqDistance < LateralThreshold){
            if (medio2->angle < -1.35)
                tiempoDormido = 250000;
            
            differentialrobot_proxy->setSpeedBase(velAvanceGiro, rot);
            usleep(tiempoDormido);  
        }else   
            if(DerDistance < LateralThreshold){
               if (ldata.front().angle > 1.35)
                   tiempoDormido = 250000;
               
                differentialrobot_proxy->setSpeedBase(velAvanceGiro, -rot);
                usleep(tiempoDormido); 
            }else
                if( FrontDistance < FrontThreshold)
                {
                    if (FrontAngle < 0){
                        differentialrobot_proxy->setSpeedBase(velAvanceGiro, rot);
                    } else{
                        differentialrobot_proxy->setSpeedBase(velAvanceGiro, -rot);
                    }
                    usleep(tiempoDormido);  
                }
                else{
                    std::initializer_list<float> menores = {DerDistance, FrontDistance, IzqDistance};
                    float velAvance = std::min(menores);
                    velAvance = velAvance * 2.5;
                    if (velAvance > 1000)
                        velAvance = 1000;
                    differentialrobot_proxy->setSpeedBase(velAvance, 0); 
        }
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}



