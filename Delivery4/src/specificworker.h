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

    //-------------ESTRUCTURAS-------------
    enum State {IDLE=1, GOTO=2, BUG=3, ALIGN=4, PARALLEL=5};
    
    /**
     * For obstacle method: Laser section to check if there are any obstacle
     */
    enum Section {LEFT = 1, RIGHT = 2, TOTAL = 3, FRONT = 4, middleLEFT = 5, middleRIGHT = 6};

    struct Pose {
        float x;
        float z;

        QVec getPose()
        {
            return QVec::vec3(x, 0, z);
        }
    };

    struct SafeBuffer {
			
        Pose targetBuffer;
        std::mutex myMutex;
        std::atomic_bool active;

        SafeBuffer() {
            active.store(false);
        }

        void push (const Pose &target) {
            std::lock_guard<std::mutex> g(myMutex);
            targetBuffer = target;
            active = true;
        }

        inline bool isActive()const
        {
            return active.load();
        }

        Pose pop () {
            std::lock_guard<std::mutex> g(myMutex);
            return targetBuffer;
        }

        void setInactive()
        {
            active.store(false);
        }

        void setActive()
        {
            active.store(true);
        }


    };
		
    //--------------VARIABLES--------------
    SafeBuffer buffer;
    InnerModel *innerModel;
    State state = IDLE;
//     State stateBefore = IDLE;
    
    Pose target;
    Pose robot;
    
    QVec vectorRobTar;
    float distance;
    float angle, angleMin, angleMax;
    
    RoboCompLaser::TLaserData ldata; //Laser divided in sections and ordered by distance
    RoboCompLaser::TLaserData ldataRaw; //Laser without modifications
    
    //When the data laser vector is ordered, these points to:
    std::vector<TData>::iterator minLeft; //Minimum distance at left
    std::vector<TData>::iterator minFront; //Minimum distance in front
    std::vector<TData>::iterator minRight; //Minimum distance at right
    std::vector<TData>::iterator middleLeft; //Minimum distance in the middleLeft
    std::vector<TData>::iterator middleRight; //Minimum distance in the middleRight 
    
    
    /**
     * -1 if the obstacle is on the left
     *  0 if the obstacle is in front
     *  1 if the obstacle is on the right     
     */
    int partObstacle=9999;
    

    //-------------CONSTANTES--------------
    const float frontThreshold = 400; //millimeters
    const float lateralThreshold = 350; //millimeters
    const float rotMax = 2; //Velocidad de rotacion máxima en rads/s
    const float advMax = 1000;
    
    //--------------METODOS-----------------
    
    
    void align();
    void gotoTarget();
    void bug();
    void parallel();
    
    //---------METODOS AUXILIARES-----------
    bool obstacle(Section section, int &partObstacle);
    bool viaLibre();
    bool targetAtSight();
    void ordenarLaser(RoboCompLaser::TLaserData ldata);

//     int n;
};

#endif
