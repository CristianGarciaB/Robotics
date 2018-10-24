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

/**
 *       \brief
 *       @author authorname
 */



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    
public:
    
    SpecificWorker(MapPrx& mprx);
    ~SpecificWorker();
    
    
    bool setParams(RoboCompCommonBehavior::ParameterList params);
    void setPick(const Pick &myPick);
    
public slots:
    
    void compute();
    
private:
    
<<<<<<< HEAD
    //DEFINICION DE ESTRUCTURAS
    enum State {IDLE=1, GOTO=2, BUG=3};
=======
    struct Target{
        float x;
        float z;
				
				QVec getPose()
				{
					return QVec::vec3(x, 0, z);
				}
    };
>>>>>>> 250dbb5bfe01d62473f82c6a6aa79c7fd740183c
    
    struct SafeBuffer {
        
        SafeBuffer() {
            activo.store(false);
        }
        
        void push (const Target &target) {
            std::lock_guard<std::mutex> g(myMutex);
            myTarget = target;
            activo = true;
        }
        
        inline bool isActive()const
        {
            return activo.load();
        }
        
        Target pop () {
            std::lock_guard<std::mutex> g(myMutex);
            return myTarget;
        }
        
        void setInactive()
        {
            activo.store(false);
        }
        
        void setActive()
        {
            activo.store(true);
        }
        
        Target myTarget;
        std::mutex myMutex;
        std::atomic_bool activo;
    };
    
    struct Target{
        float x;
        float z;
        
        QVec getPose()
        {
            return QVec::vec3(x, 0, z);
        }
    };
    
    
    //VARIABLES
    
    SafeBuffer buffer;
    
    InnerModel *innerModel;
    
    State state = IDLE; //inicializado a IDLE 
    
    //Para obstacle ()
    const float FrontThreshold = 335; //millimeters
	const float LateralThreshold = 235; //millimeters
    float rot = 0.9;  //Velocidad de rotacion rads per second
    
    
     //MODULOS
    void gotoTarget();
    void bug();
    bool obstacle();
    bool targetAtSight(RoboCompLaser::TLaserData laserData, Target target);
    

};

#endif
