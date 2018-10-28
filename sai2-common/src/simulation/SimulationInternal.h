/*
 * SimulationInternal.h
 *
 *  Created on: Dec 27, 2016
 *      Author: Shameek Ganguly
 */

#ifndef SIMULATION_INTERNAL_H
#define SIMULATION_INTERNAL_H

#include "SimulationInterface.h"

namespace Simulation {

class SimulationInternal {
protected:
	/**
     * @brief Default constructor is suppressed by default since this class has no in-built functionality.
     */
    SimulationInternal(){;}

public:
	/**
     * @brief Get degrees of freedom of a particular robot. NOTE: Assumes serial chain robot.
     * @param robot_name Name of the robot for which transaction is required.
     */
	virtual unsigned int dof(const std::string& robot_name) const = 0;

	/**
     * @brief Set joint positions as an array. Assumes serial chain robot.
     * @param robot_name Name of the robot for which transaction is required.
     * @param q Desired joint position values.
     */
	virtual void setJointPositions(const std::string& robot_name,
									const Eigen::VectorXd& q) = 0;

	/**
     * @brief Read back joint positions as an array.
     * @param robot_name Name of the robot for which transaction is required.
     * @param q_ret Array to write back joint positions into.
     */
	virtual void getJointPositions(const std::string& robot_name,
									Eigen::VectorXd& q_ret) const = 0;

	/**
     * @brief Set joint position for a single joint
     * @param robot_name Name of the robot for which transaction is required.
     * @param joint_id Joint number on which to set value.
     * @param position Value to set.
     */
	virtual void setJointPosition(const std::string& robot_name,
									unsigned int joint_id,
									double position) = 0;
	
	/**
     * @brief Set joint velocities as an array. Assumes serial chain robot.
     * @param robot_name Name of the robot for which transaction is required.
     * @param dq Desired joint velocity values.
     */
	virtual void setJointVelocities(const std::string& robot_name,
									const Eigen::VectorXd& dq) = 0;

	/**
     * @brief Set joint velocity for a single joint
     * @param robot_name Name of the robot for which transaction is required.
     * @param joint_id Joint number on which to set value.
     * @param velocity Value to set.
     */
	virtual void setJointVelocity(const std::string& robot_name,
									unsigned int joint_id,
									double velocity) = 0;

	/**
     * @brief Read back joint velocities as an array.
     * @param robot_name Name of the robot for which transaction is required.
     * @param dq_ret Array to write back joint velocities into.
     */
	virtual void getJointVelocities(const std::string& robot_name,
									Eigen::VectorXd& dq_ret) const = 0;

	/**
     * @brief Set joint torques as an array. Assumes serial chain robot.
     * @param robot_name Name of the robot for which transaction is required.
     * @param tau Desired joint torque values.
     */
	virtual void setJointTorques(const std::string& robot_name,
									const Eigen::VectorXd& tau) = 0;

	/**
     * @brief Set joint torque for a single joint
     * @param robot_name Name of the robot for which transaction is required.
     * @param joint_id Joint number on which to set value.
     * @param tau Value to set.
     */
	virtual void setJointTorque(const std::string& robot_name,
								unsigned int joint_id,
								double tau) = 0;

	/**
     * @brief Read back joint torques as an array.
     * @param robot_name Name of the robot for which transaction is required.
     * @param tau_ret Array to write back joint torques into.
     */
	virtual void getJointTorques(const std::string& robot_name,
									Eigen::VectorXd& tau_ret) const = 0;

	/**
     * @brief Read back joint accelerations as an array.
     * @param robot_name Name of the robot for which transaction is required.
     * @param ddq_ret Array to write back joint accelerations into.
     */
	virtual void getJointAcclerations(const std::string& robot_name,
										Eigen::VectorXd& ddq_ret) const = 0;

	/**
     * @brief Integrate the virtual world over given time step.
     * @param timestep Time step in seconds by which to forward simulation.
     */
	virtual void integrate(double timestep) = 0;

     /**
      * @brief      Shows the contact information, whenever a contact occurs
      */
     virtual void showContactInfo() = 0;

     /**
      * @brief      Gets a vector of contact points and the corresponding contact forces at a gicen link of a given object. Gives everything in base frame.
      *
      * @param      contact_points  The contact points vector to be returned
      * @param      contact_forces  The contact forces vector to be returned
      * @param[in]  robot_name      The robot name
      * @param[in]  link_name       The link name
      */
     virtual void getContactList(std::vector<Eigen::Vector3d>& contact_points, std::vector<Eigen::Vector3d>& contact_forces, 
     const::std::string& robot_name, const std::string& link_name) = 0;
};

}

#endif //SIMULATION_INTERNAL_H
