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

}

void SpecificWorker::turn(const float speed)
{
//implementCODE

}

bool SpecificWorker::atTarget()
{
//implementCODE

}

void SpecificWorker::stop()
{
//implementCODE

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
	planReady = false;
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


