/**
 * \file GraphicsInterface.cpp
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#include "GraphicsInterface.h"
#include "ChaiGraphics.h"
#include "model/ModelInterface.h"

namespace Graphics {

GraphicsInterface::GraphicsInterface(const std::string& path_to_world_file,
										Graphics::GraphicsType graphics_type,
										Graphics::ParserType parser,
										bool verbose)
: _graphics_internal(NULL) {
	// initialize internal model depending on what type of model and parser are
	// chosen by user
	switch (graphics_type) {
		case chai:
			_graphics_internal = new ChaiGraphics(
					path_to_world_file,
            		parser,
            		verbose);
			break;
	}
}

// dtor
GraphicsInterface::~GraphicsInterface() {
	delete _graphics_internal;
	_graphics_internal = NULL;
}

// update frame for a particular robot
void GraphicsInterface::updateGraphics(const std::string& robot_name,
										Model::ModelInterface* robot_model) {
	_graphics_internal->updateGraphics(robot_name, robot_model);
}

// render from a camera
void GraphicsInterface::render(const std::string& camera_name,
								int window_width, 
								int window_height, 
								int display_context_id) {
	_graphics_internal->render(camera_name, window_width, window_height, display_context_id);
}

// get current camera pose
void GraphicsInterface::getCameraPose(const std::string& camera_name,
				                         Eigen::Vector3d& ret_position,
				                         Eigen::Vector3d& ret_vertical,
				                         Eigen::Vector3d& ret_lookat) {
	_graphics_internal->getCameraPose(camera_name, ret_position, ret_vertical, ret_lookat);
}

// set camera pose
 void GraphicsInterface::setCameraPose(const std::string& camera_name,
										const Eigen::Vector3d& position,
										const Eigen::Vector3d& vertical,
										const Eigen::Vector3d& lookat) {
 	_graphics_internal->setCameraPose(camera_name, position, vertical, lookat);
}

bool GraphicsInterface::getRobotLinkInCamera(const std::string& camera_name,
			                                   const std::string& robot_name,
			                                   int view_x,
			                                   int view_y,
			                                   int window_width,
			                                   int window_height,
			                                   std::string& ret_link_name,
			                                   Eigen::Vector3d& ret_pos) {
	return _graphics_internal->getRobotLinkInCamera(camera_name, robot_name, view_x, view_y, window_width, window_height, ret_link_name, ret_pos);
}

}
