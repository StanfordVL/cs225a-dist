/**
 * \file CRobotBase.h
 *
 * \brief This file is part of the extended chai functionality to support
 * articulated rigid body structures.
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#ifndef CRobotBaseH
#define CRobotBaseH

#include "chai3d.h"

namespace chai3d {

class cRobotBase: public cGenericObject {
public:
	/**
     * @brief Creates a cRobotBase object. This is a simply a type 
     * specialization for cGenericObject to be used for robot bases.
     */
	cRobotBase() {;}
};

}

#endif //CRobotBaseH