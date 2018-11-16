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
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include "grid.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <iostream>
#include <fstream>

class SpecificWorker : public GenericWorker
{
	Q_OBJECT
	public:
		SpecificWorker(MapPrx& mprx);
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		
		// Ice subscription
		void setPick(const Pick &myPick);

	public slots:
		void compute();
		void saveToFile();

	private:
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
		
		SafeBuffer buffer;
		Pose target;
		std::shared_ptr<InnerModel> innerModel;
		QGraphicsScene scene;
		QGraphicsView view;
		void draw();
		QGraphicsRectItem *robot;
		QGraphicsEllipseItem *noserobot;
		QVec targetVector;
		
		/*
		using nodo = std::tuple<Key, float>;
		
		std::vector<nodo> abiertos;
		std::vector<nodo> cerrados;
		*/
		void updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata);
		void checkTransform(const RoboCompGenericBase::TBaseState &bState);
		
		/// Grid
		struct TCell
		{
			bool free;
			bool visited;
			QGraphicsRectItem* rect;
			
			// method to save the value
			void save(std::ostream &os) const {	os << free << " " << visited; };
			void read(std::istream &is) {	is >> free >> visited ;};
		};
		
		using TDim = Grid<TCell>::Dimensions;
		Grid<TCell> grid;
		

};

#endif
