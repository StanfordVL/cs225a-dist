/**
 * \file ChaiGraphics.cpp
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#include "ChaiGraphics.h"
#include "model/ModelInterface.h"
#include "parser/URDFToChaiGraphics.h"
#include <iostream>

using namespace std;
using namespace chai3d;

namespace Graphics {

ChaiGraphics::ChaiGraphics(const std::string& path_to_world_file,
							Graphics::ParserType parser,
							bool verbose)
: _world(NULL) {
	// initialize a chai world
	_world = new chai3d::cWorld();
	switch(parser) {
		case Graphics::urdf:
			Parser::URDFToChaiGraphicsWorld(path_to_world_file, _world, verbose);
			break;
	}
}

// dtor
ChaiGraphics::~ChaiGraphics() {
	delete _world;
	_world = NULL;	
}

static void updateGraphicsLink(cRobotLink* link, Model::ModelInterface* robot_model) {
	cVector3d local_pos;
	cMatrix3d local_rot;

	// get link name
	std::string link_name = link->m_name;

	// get chai parent name
	cGenericObject* parent = link->getParent();
	std::string parent_name = parent->m_name;

	// if parent is cRobotBase, then simply get transform relative to base from model
	if (dynamic_cast<cRobotBase*> (parent) != NULL) {
		Eigen::Affine3d T;
		robot_model->transform(T, link_name);
		local_pos = cVector3d(T.translation());
		local_rot = cMatrix3d(T.rotation());
	} else if (dynamic_cast<cRobotLink*> (parent) != NULL) {
		// if parent is cRobotLink, then calculate transform for both links, then apply inverse transform
		Eigen::Affine3d T_me, T_parent, T_rel;
		robot_model->transform(T_me, link_name);
		robot_model->transform(T_parent, parent_name);
		T_rel = T_parent.inverse() * T_me;
		local_pos = cVector3d(T_rel.translation());
		local_rot = cMatrix3d(T_rel.rotation());
	} else {
		cerr << "Parent to link " << link_name << " is neither link nor base" << endl;
		abort();
		// TODO: throw exception
	}
	link->setLocalPos(local_pos);
	link->setLocalRot(local_rot);

	// call on children
	cRobotLink* child;
	for (unsigned int i = 0; i < link->getNumChildren(); ++i) {
		child = dynamic_cast<cRobotLink*>(link->getChild(i));
		if (child != NULL) {
			updateGraphicsLink(child, robot_model);
		}
	}
}

// update frame for a particular robot
void ChaiGraphics::updateGraphics(const std::string& robot_name,
									Model::ModelInterface* robot_model) {
	// get robot base object in chai world
	cRobotBase* base = NULL;
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		if (robot_name == _world->getChild(i)->m_name) {
			// cast to cRobotBase
			base = dynamic_cast<cRobotBase*>(_world->getChild(i));
			if (base != NULL) {
				break;
			}
		}
	}
	if (base == NULL) {
		//TODO: throw exception
		cerr << "Could not find robot in chai world: " << robot_name << endl;
		abort();
	}
	// recursively update graphics for all children
	cRobotLink* link;
	for (unsigned int i = 0; i < base->getNumChildren(); ++i) {
		link = dynamic_cast<cRobotLink*>(base->getChild(i));
		if (link != NULL) {
			updateGraphicsLink(link, robot_model);
		}
	}
	// update shadow maps. TODO: consider moving out from here if it is too expensive
	_world->updateShadowMaps();
}

void ChaiGraphics::render(const std::string& camera_name,
							int window_width, 
							int window_height, 
							int display_context_id) {
	auto camera = getCamera(camera_name);
	// TODO: support link mounted cameras
	// TODO: support stereo. see cCamera::renderView
	//	to do so, we need to search through the descendent tree
	// render view from this camera
	// NOTE: we don't use the display context id right now since chai no longer
	// supports it in 3.2.0
	camera->renderView(window_width, window_height);
}

// get current camera pose
void ChaiGraphics::getCameraPose(const std::string& camera_name,
									Eigen::Vector3d& ret_position,
									Eigen::Vector3d& ret_vertical,
									Eigen::Vector3d& ret_lookat) {
	auto camera = getCamera(camera_name);
	cVector3d pos, vert, lookat;
	pos = camera->getLocalPos(); ret_position << pos.x(), pos.y(), pos.z();
	vert = camera->getUpVector(); ret_vertical << vert.x(), vert.y(), vert.z();
	lookat = camera->getLookVector(); ret_lookat << lookat.x(), lookat.y(), lookat.z();
}

// set camera pose
void ChaiGraphics::setCameraPose(const std::string& camera_name,
									const Eigen::Vector3d& position,
									const Eigen::Vector3d& vertical,
									const Eigen::Vector3d& lookat) {
 	auto camera = getCamera(camera_name);
	cVector3d pos(position[0], position[1], position[2]);
	cVector3d vert(vertical[0], vertical[1], vertical[2]);
	cVector3d look(lookat[0], lookat[1], lookat[2]);
	camera->set(pos, look, vert);
}

bool ChaiGraphics::getRobotLinkInCamera(const std::string& camera_name,
		                                   const std::string& robot_name,
		                                   int view_x,
		                                   int view_y,
		                                   int window_width,
										   int window_height,
		                                   std::string& ret_link_name,
		                                   Eigen::Vector3d& ret_pos) {
	cCollisionRecorder selectionRecorder;

	//use standard settings
	cCollisionSettings defaultSettings;
	defaultSettings.m_checkForNearestCollisionOnly = false;
	defaultSettings.m_checkVisibleObjects = true;
	defaultSettings.m_collisionRadius = 0;
	defaultSettings.m_returnMinimalCollisionData = false;

	//use collision detection on the present camera!
	bool hit = getCamera(camera_name)->selectWorld(
										view_x,
										view_y,
										window_width,
										window_height,
										selectionRecorder,
										defaultSettings);
	if(hit) {
		cVector3d pos = selectionRecorder.m_nearestCollision.m_localPos;
		auto object = selectionRecorder.m_nearestCollision.m_object;
		if(object->getParent() == NULL) {
			return false;
		}
		auto object_parent = object->getParent();
		while (object_parent != NULL) {
			if (robot_name == object_parent->m_name) {
				ret_link_name = object->getParent()->m_name;
				ret_pos << pos.x(), pos.y(), pos.z();
				return true;
			}
			object_parent = object_parent->getParent();
		}
	}
	return false;
}

// get camera object
cCamera* ChaiGraphics::getCamera(const std::string& camera_name) {
	cCamera* camera;
	// find camera among children
	// TODO: this can be optimized by locally storing list of cameras
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		cGenericObject* child = _world->getChild(i);
		if (camera_name == child->m_name) {
			camera = dynamic_cast<cCamera*>(child);
			if (camera) {
				break;
			}
		}
	}
	if (!camera) {
		cerr << "Could not find camera named " << camera_name << endl;
		abort();
		//TODO: throw exception instead
	}
	return camera;
}

}
