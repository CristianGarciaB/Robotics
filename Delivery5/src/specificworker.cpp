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
#include "math.h"
/**
 * \brief Default constructor
 */
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	std::cout << std::boolalpha;   
}

/**
 * \brief Default destructor
 */
SpecificWorker::~SpecificWorker()
{}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		innerModel = std::make_shared<InnerModel>(par.value);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }
	
	qDebug() << __FILE__ ;
	
	// Scene
	scene.setSceneRect(-2500, -2500, 5000, 5000);
	view.setScene(&scene);
	view.scale(1, -1);
	view.setParent(scrollArea);
	//view.setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
	
	//choose here or create a button in the UI to load from file
	//grid.initialize( TDim{ tilesize, -2500, 2500, -2500, 2500}, TCell{true, false, nullptr} );
	grid.readFromFile("map.txt");
	
	for(auto &[key, value] : grid)
	{
		auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));
		tile->setPos(key.x,key.z);
		value.rect = tile;
	}
	
	robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
	noserobot = new QGraphicsEllipseItem(-50,100, 100,100, robot);
	noserobot->setBrush(Qt::magenta);
	
	//target = QVec::vec3(0,0,0);
	
	//qDebug() << __FILE__ << __FUNCTION__ << "CPP " << __cplusplus;
	
	connect(pushButton, SIGNAL(clicked()), this, SLOT(saveToFile()));
	timer.start();
	
	return true;
}

void SpecificWorker::compute()
{
	static RoboCompGenericBase::TBaseState bState;
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		
		//draw robot
		robot->setPos(bState.x, bState.z);
		robot->setRotation(-180.*bState.alpha/M_PI);
		
		//updateOccupiedCells(bState, ldata);
		
		if(buffer.isActive())
		{
			target = buffer.pop();
			buffer.setInactive();
			
			abiertos.clear();
			cerrados.clear();
			path.clear();
			
			nodo inicial2 (NULL, Key(bState.x, bState.z), TCell{true, false, nullptr}, 0, 0);
			inicial=inicial2;
			abiertos.push_back(inicial);
			
			aEstrella();
		}
		
	}
	catch(const Ice::Exception &e)
	{	std::cout  << e << std::endl; }
	
	//Resize world widget if necessary, and render the world
	if (view.size() != scrollArea->size())
		view.setFixedSize(scrollArea->width(), scrollArea->height());
	draw();
	
}

bool SpecificWorker::enListaCerrados(nodo a)
{
	bool inList = false;
	
	for (auto l : cerrados)
	{
		if (l == a)
		{
			inList = true;
			break;
		}
	}
	return inList;
}

bool SpecificWorker::enPath(Key k)
{
	bool inList = false;
	
	for (auto l : path)
	{
		if (l.k == k)
		{
			inList = true;
			break;
		}
	}
	return inList;
}

bool SpecificWorker::enListaAbiertos(nodo a, nodo *enLista)
{
	bool inList = false;
	
	for (auto l : abiertos)
	{
		if (l == a)
		{
			inList = true;
			*enLista = l;
			break;
		}
	}
	
	return inList;
}

float SpecificWorker::calcularFuncion(float cost, Key k)
{
	float valor, coste, heuristica;
	
	coste = cost + 1;
	heuristica = sqrt(pow((target.x - k.x),2) + pow((target.z - k.z),2));
	valor = coste + heuristica;
	
	return valor;
	
}

void SpecificWorker::getPath()
{
	nodo anterior = cerrados.back();
	path.push_back(anterior);
	
	while(!(anterior == inicial))
	{
		anterior = *anterior.padre;
		path.insert(path.begin(), anterior);
	}
}

bool SpecificWorker::aEstrella()
{
 	bool pathFound = false;
	nodo menor;
	nodo targetNode (Key(target.x, target.z));

	abiertos.clear();
	nodo a (NULL, Key(-50, 20), TCell{true, false, nullptr}, 0, 0);
	abiertos.push_back(a);
	
	while(!pathFound and !abiertos.empty())
	{
		std::sort(abiertos.begin(), abiertos.end(), [](nodo a, nodo b) {
        return a < b;   
    });
		
		menor = abiertos.front();
		cerrados.push_back(menor);
		abiertos.erase(abiertos.begin());
			
		if (menor == targetNode)
			pathFound = true;
		else
		{
			const std::vector<std::pair<Key,TCell>> vecinos = grid.neighbours(menor.k);
			for (auto &[k, v] : vecinos)
			{
				nodo nuevo (&menor, k, v, calcularFuncion(menor.c, k), menor.c+1);
				if(!enListaCerrados(nuevo) && v.free)
				{
					nodo enLista;
					if(!enListaAbiertos(nuevo, &enLista)){
						abiertos.push_back(nuevo);
					}else{
						if(nuevo.c < enLista.c)
						{
							enLista.padre = &menor;
							enLista.c = nuevo.c;
							enLista.f = nuevo.f;
						}
					}
				}
			}
		}
	}
	
	if(pathFound)
		getPath();
	
	return pathFound;
}

void SpecificWorker::saveToFile()
{
	std::ofstream myfile;
	myfile.open ("map.txt");
	myfile << tilesize << std::endl;
	for(auto &[k, v] : grid)
	{
		myfile << k << v << std::endl;
	}
	myfile.close();
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
		if(value.visited == false)
			value.rect->setBrush(Qt::lightGray);
		if(value.free == false)
			value.rect->setBrush(Qt::darkRed);
		if(enPath(key))
			value.rect->setBrush(Qt::green);
			
	}
	view.show();
}

/////////////////////////////////////////////////////////77
/////////
//////////////////////////////////////////////////////////

void SpecificWorker::setPick(const Pick &myPick)
{
	Pose target;
	target.x = myPick.x;
	target.z = myPick.z;
	buffer.push ( target );
}
