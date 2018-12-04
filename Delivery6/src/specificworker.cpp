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
    //cuando el estado es goto tiene que pasar al metodo go 
    //cuando vuelve del atTarget con TRUE espera una siguiente marca (target) que se la envia el mission planner
    
    //........................................................//
    
    static RoboCompGenericBase::TBaseState bState;
    try
    {
    
        differentialrobot_proxy->getBaseState(bState);
        innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
        
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();        
        
        if(targetReady){ //cuando recibe un target del mission planner esta a true
            switch (state){
                case State::IDLE: {
                    if (atTarget()){ //si ha llegado al target
                        std::cout << "AT TARGET. STOP." << std::endl;
                        differentialrobot_proxy->stopBase(); 
                        targetReady = false;
                    }
                    else { //si no ha llegado al target, tiene que seguir hacia él
                        state = State::GOTO;
                    }                        
                }break;
                
                case State::GOTO: { //va hacia el target
                    
                    std::cout << "GO TO THE TARGET" << std::endl;
                    
                    //TODO meter parametros
//                     go();   
                    
                }break;
                
                case State::ALIGN: {

                    
                    QVec r = innerModel->transform(
                    "base", QVec::vec3(target.x(), 0, target.z()), "world");
                    
                    distancia = r.norm2();
                    angulo = atan2(r.z(), r.x());

                    align(); 
                    //ya ha sido alineado
                    differentialrobot_proxy->setSpeedBase(0, k*rotMax);

                }break;
                
            }
        }
        
        else{ //ya no hay mas target en el mission planner
            std::cout << "There aren't more aprilTags to visit" << std::endl;
            differentialrobot_proxy->stopBase();
            
            //guardar las marcas en una lista que tiene el mission planner y este le envia al robot el siguiente target
            
            //aqui es donde el mission planner tiene que mandarle una marca
            //target = lo que le mande el mission planner
            
            //cuando hay una nueva marca pasa al goto
            targetReady = true;
            state = State::GOTO;
    
     
        }
        
    }
    catch(const Ice::Exception &e)
    {	std::cout  << e << std::endl; }
    
	QMutexLocker locker(mutex);
	//computeCODE
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}
}


void SpecificWorker::go(const string &nodo, const float x, const float y, const float alpha)
{
//implementCODE
    
    //copiamos lo de la practica anterior para moverse (con alguna modificacion)

}

void SpecificWorker::turn(const float speed)
{
//implementCODE

}

bool SpecificWorker::atTarget()
{
    bool arrived = false;
    
    if (distancia < 100){
        arrived = true;      
    }
    
    return arrived;

}

void SpecificWorker::align(){
                        
    if (angulo < 1.63 && angulo > 1.51) //Si ya está alineado
    {
        state = State::GOTO;
        return;
    }
    
    if(angulo > 0.57 && angulo < 2.57)
        k = pow((angulo - 1.57), 2) + 0.35;
    else
        k = 1;
    
    if (abs(angulo) > 1.57)
        k = -1;
}

void SpecificWorker::stop()
{
//implementCODE
    //espera un nuevo target

}


//METODOS A LOS QUE ESTA SUSCRITO, SOLO TENEMOS QUE MOSTRAR PARA SABER QUE LOS PILLA BIEN

void SpecificWorker::setPick(const Pick &myPick)
{
//subscribesToCODE
    target[0] = myPick.x;
	target[2] = myPick.z;
	target[1] = 0;
	qDebug() << __FILE__ << __FUNCTION__ << myPick.x << myPick.z ;
	targetReady = true;
// 	planReady = false;
	for(auto gp: greenPath)
		delete gp;
	greenPath.clear();

}

void SpecificWorker::newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState)
{
//subscribesToCODE
//NO HAY QUE ESCRIBIR NADA DE CODIGO, NO SE COMENTA PARA QUE NO PETE

}

//imprime las variables de la lista de tags que hay en el aprilTag que apunta
void SpecificWorker::newAprilTag(const tagsList &tags)
{
//subscribesToCODE
    for(auto t:tags)
        std::cout<<t.id<<" "<<t.tx<<" "<<t.ty<<" "<<t.tz<<std::endl;

}


