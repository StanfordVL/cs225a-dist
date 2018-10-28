/**
 * \file CRobotLink.h
 *
 * \brief This file is part of the extended chai functionality to support
 * articulated rigid body structures.
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#ifndef CRobotLinkH
#define CRobotLinkH

#include "chai3d.h"

namespace chai3d {

class cRobotLink: public cGenericObject {
public:
	/**
     * @brief Creates a cRobotLink object. This is a simply a type 
     * specialization for cGenericObject to be used for robot links.
     */
	cRobotLink() {;}
};

}

#endif //CRobotLinkH