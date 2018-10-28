/**
 * \file GraphicsInternal.h
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#ifndef GRAPHICS_INTERNAL_H
#define GRAPHICS_INTERNAL_H

#include "GraphicsInterface.h"

namespace Graphics {

class GraphicsInternal {
protected:
	// default ctor is hidden away
	GraphicsInternal() {;}

public:
	/// \brief Virtual destructor
	virtual ~GraphicsInternal() {;}

	/**
     * @brief Update the graphics model for a robot in the virtual world.
     * @param robot_name Name of the robot for which model update is considered.
     * @param robot_model A ModelInterface object for this robot to be used to get intermediate transforms.
     */
	virtual void updateGraphics(const std::string& robot_name,
								Model::ModelInterface* robot_model) = 0;

	/**
     * @brief Render the virtual world to the current context.
     * 	NOTE: the correct context should have been selected prior to this.
     * @param camera_name Camera name to be rendered from.
     * @param window_width Width of viewport in screen co-ordinates.
     * @param window_height Height of viewport in screen co-ordinates.
     * @param display_context_id ID for the context to display in. This ID is only to be used
     *	for selective rendering. It does not change the GL context to render to.
     */
	virtual void render(const std::string& camera_name,
						int window_width, 
						int window_height, 
						int display_context_id = 0) = 0;

	/**
     * @brief Return the pose of the camera in the parent frame
     * @param camera_name Camera name.
     * @param ret_position Position of the camera.
     * @param ret_vertical Up vector for the camera.
     * @param ret_lookat Point the camera is looking at.
     */
	virtual void getCameraPose(const std::string& camera_name,
								Eigen::Vector3d& ret_position,
								Eigen::Vector3d& ret_vertical,
								Eigen::Vector3d& ret_lookat) = 0;

	/**
     * @brief Sets the pose of the camera in the parent frame
     * @param camera_name Camera name.
     * @param position Position of the camera.
     * @param vertical Up vector for the camera.
     * @param lookat Point the camera is to look at.
     */
    virtual void setCameraPose(const std::string& camera_name,
								const Eigen::Vector3d& position,
								const Eigen::Vector3d& vertical,
								const Eigen::Vector3d& lookat) = 0;

    /**
     * @brief Get info about link of the specified robot at the given cursor position.
     * @return True if a link is present, False if not
     * @param camera_name Camera name.
     * @param robot_name Name of robot to look for.
     * @param view_x x-position of cursor in viewport (OpenGL style screen co-ordinates).
     * @param view_y y-position of cursor in viewport (OpenGL style screen co-ordinates).
     * @param window_width Width of viewport in screen co-ordinates.
     * @param window_height Height of viewport in screen co-ordinates.
     * @param ret_link_name Name of the link. Garbage if no link present at cursor location.
     * @param ret_pos Position of cursor in link frame. Garbage if no link present at cursor location.
     */
    virtual bool getRobotLinkInCamera(const std::string& camera_name,
	                                   const std::string& robot_name,
	                                   int view_x,
	                                   int view_y,
	                                   int window_width,
									   int window_height,
	                                   std::string& ret_link_name,
	                                   Eigen::Vector3d& ret_pos) {
    	return false;
    }
};

}

#endif //GRAPHICS_INTERNAL_H
