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
		innerModel = std::make_shared<InnerModel>(par.value);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }
	
	qDebug() << __FILE__ ;
	
	targetReady.store(false);
	planReady.store(false);
	
	// Scene
	scene.setSceneRect(-2500, -2500, 5000, 5000);
	view.setScene(&scene);
	view.scale(1, -1);
	view.setParent(scrollArea);
	//view.setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
	
	grid.initialize( TDim{ tilesize, -2500, 2500, -2500, 2500}, TCell{0, true, false, nullptr, 0.} );
	
	for(auto &[key, value] : grid)
	{
		auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));
		tile->setPos(key.x,key.z);
		value.rect = tile;
	}
	
	robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
	noserobot = new QGraphicsEllipseItem(-50,100, 100,100, robot);
	noserobot->setBrush(Qt::magenta);
	
	target = QVec::vec3(0,0,0);
	
	//qDebug() << __FILE__ << __FUNCTION__ << "CPP " << __cplusplus;
	
	connect(buttonSave, SIGNAL(clicked()), this, SLOT(saveToFile()));
	connect(buttonRead, SIGNAL(clicked()), this, SLOT(readFromFile()));
	
	timer.start();
	
	isAtTarget.store(true);
	
	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	static RoboCompGenericBase::TBaseState bState;
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		
		//draw robot
		robot->setPos(bState.x, bState.z);
		robot->setRotation(-180.*bState.alpha/M_PI);
		
		updateOccupiedCells(bState, ldata);
		
		if(targetReady)
		{
			if(planReady)
			{
				if(path.empty())
				{
					qDebug() << "Arrived to target";
					differentialrobot_proxy->stopBase();
					targetReady = false; 
					isAtTarget.store(true);
				}
				else
					if(innerModel->transform("base", QVec::vec3(currentPoint.x(), 0, currentPoint.z()),"world").norm2() < 150)
					{
						currentPoint = path.front();
						path.pop_front();
						state = State::ALIGN;
					}
					else
					{
						
						QVec r = innerModel->transform("base", QVec::vec3(currentPoint.x(), 0, currentPoint.z()),"world");
						float distancia = r.norm2();
						float angulo = atan2(r.z(), r.x());
						float rotMax = 0.75;
						//                         float advMax = 1000;
						float k = 1;
						
						switch(state){
							
							case State::ALIGN:{
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
								
								differentialrobot_proxy->setSpeedBase(0, k*rotMax);
							}break;
							
							case State::GOTO:{
								QVec r = innerModel->transform("base", QVec::vec3(target.x(), 0, target.z()),"world");
								
								
								if (distancia < 100)
								{
									targetReady = false;
									differentialrobot_proxy->stopBase();
									state = State::ALIGN;
									return;
								}
								
								
								differentialrobot_proxy->setSpeedBase(300, 0);
								
							}break;
						}
					}
			}
			else
			{
				qDebug() <<"-------------------------------------------------------";
				qDebug() <<"Coordinates robot (real): "<< bState.x << bState.z;
				qDebug() <<"Coordinates target (real): " << target.x() << target.z() ;
				path = grid.getOptimalPath(QVec::vec3(bState.x,0,bState.z), target);
				for(auto &p: path){
					greenPath.push_back(scene.addEllipse(p.x(),p.z(), 100, 100, QPen(Qt::green), QBrush(Qt::green)));
				}
				planReady = true;
				
			}
		}
		
	}
	catch(const Ice::Exception &e)
	{	std::cout  << e << std::endl; }
	
	//Resize world widget if necessary, and render the world
	if (view.size() != scrollArea->size())
		view.setFixedSize(scrollArea->width(), scrollArea->height());
	draw();
	
}

void SpecificWorker::go(const string &nodo, const float x, const float y, const float alpha)
{
	target[0] = x;
	target[2] = y;
	planReady.store(false);
	targetReady.store(true);
	isAtTarget.store(false);
}

void SpecificWorker::turn(const float speed)
{
	//implementCODE
	
}

bool SpecificWorker::atTarget()
{ 
	return isAtTarget.load();
}

void SpecificWorker::stop()
{
	//implementCODE
	//espera un nuevo target
	
}


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
}

//imprime las variables de la lista de tags que hay en el aprilTag que apunta
void SpecificWorker::newAprilTag(const tagsList &tags)
{
	//subscribesToCODE
	std::cout<<"Tags read:"<<std::endl;
	for(auto t : tags)
		std::cout<<"ID: "<<t.id<<" X: "<<t.tx<<" Y: "<<t.ty<<" Z: "<<t.tz<<std::endl;
}

void SpecificWorker::saveToFile()
{
	grid.saveToFile(fileName);
}

void SpecificWorker::readFromFile()
{
	std::ifstream myfile;
	myfile.open(fileName, std::ifstream::in);
	
	if(!myfile.fail())
	{
		//grid.initialize( TDim{ tilesize, -2500, 2500, -2500, 2500}, TCell{true, false, nullptr} );
		for( auto &[k,v] : grid)
			delete v.rect;
		grid.clear();
		Grid<TCell>::Key key; TCell value;
		myfile >> key >> value;
		int k=0;
		while(!myfile.eof()) 
		{
			auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));;
			tile->setPos(key.x,key.z);
			value.rect = tile;
			value.id = k++;
			value.cost = 1;
			grid.insert<TCell>(key,value);
			myfile >> key >> value;
		}
		myfile.close();	
		robot->setZValue(1);
		std::cout << grid.size() << " elements read to grid " << fileName << std::endl;
	}
	else
		throw std::runtime_error("Cannot open file");
}

void SpecificWorker::updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)
{
	auto *n = innerModel->getNode<InnerModelLaser>("laser");
	for(auto l: ldata)
	{
		auto r = n->laserTo("world", l.dist, l.angle);	// r is in world reference system
		// we set the cell corresponding to r as occupied 
		auto [valid, cell] = grid.getCell(r.x(), r.z()); 
		if(valid)
			cell.free = false;
	}
}

void SpecificWorker::draw()
{
	for(auto &[key, value] : grid)
	{
		// 		if(value.visited == false)
		// 			value.rect->setBrush(Qt::lightGray);
		if(value.free == false)
			value.rect->setBrush(Qt::darkRed);
	}
	view.show();
}

