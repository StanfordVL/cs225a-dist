/*
 * SimulationInterface.h
 *
 *  Created on: Dec 27, 2016
 *      Author: Shameek Ganguly
 */

#ifndef SIMULATION_INTERFACE_H
#define SIMULATION_INTERFACE_H

#include <string>
#include <vector>
#include <Eigen/Core>

/** @namespace Simulation
 *	@brief Namespace for various simulation engine interfaces.
 */
namespace Simulation
{

/// \brief Parser types supported by this interface
enum ParserType {yml, urdf};

/// \brief Simulation engines supported by this interface
enum SimulationType {sai2simulation};

// forward define
class SimulationInternal;

class SimulationInterface {
public:
	/**
     * @brief Creates a simulation interface object that contains a model of the virtual world, run by a dynamics engine.
     * @param path_to_world_file A path to the file containing the model of the virtual world (urdf and yml files supported).
     * @param simulation_type The dynamics engine we want to use (Dynamics3d supported only for now).
     * @param parser The type of parser to use in function of the file type (urdf or yml).
     * @param verbose To display information about the robot model creation in the terminal or not.
     */
	SimulationInterface(const std::string& path_to_world_file,
                		Simulation::SimulationType simulation_type,
                		Simulation::ParserType parser,
                		bool verbose = false);

	/// \brief destructor to clean up internal model
	~SimulationInterface();

	/**
     * @brief Get degrees of freedom of a particular robot. NOTE: Assumes serial chain robot.
     * @param robot_name Name of the robot for which transaction is required.
     */
	unsigned int dof(const std::string& robot_name) const;

	/**
     * @brief Set joint positions as an array. Assumes serial chain robot.
     * @param robot_name Name of the robot for which transaction is required.
     * @param q Desired joint position values.
     */
	void setJointPositions(const std::string& robot_name,
							const Eigen::VectorXd& q);

	/**
     * @brief Read back joint positions as an array.
     * @param robot_name Name of the robot for which transaction is required.
     * @param q_ret Array to write back joint positions into.
     */
	void getJointPositions(const std::string& robot_name,
							Eigen::VectorXd& q_ret) const;

	/**
     * @brief Set joint position for a single joint
     * @param robot_name Name of the robot for which transaction is required.
     * @param joint_id Joint number on which to set value.
     * @param position Value to set.
     */
	void setJointPosition(const std::string& robot_name,
							unsigned int joint_id,
							double position);
	
	/**
     * @brief Set joint velocities as an array. Assumes serial chain robot.
     * @param robot_name Name of the robot for which transaction is required.
     * @param dq Desired joint velocity values.
     */
	void setJointVelocities(const std::string& robot_name,
							const Eigen::VectorXd& dq);

	/**
     * @brief Set joint velocity for a single joint
     * @param robot_name Name of the robot for which transaction is required.
     * @param joint_id Joint number on which to set value.
     * @param velocity Value to set.
     */
	void setJointVelocity(const std::string& robot_name,
							unsigned int joint_id,
							double velocity);

	/**
     * @brief Read back joint velocities as an array.
     * @param robot_name Name of the robot for which transaction is required.
     * @param dq_ret Array to write back joint velocities into.
     */
	void getJointVelocities(const std::string& robot_name,
							Eigen::VectorXd& dq_ret) const;

	/**
     * @brief Set joint torques as an array. Assumes serial chain robot.
     * @param robot_name Name of the robot for which transaction is required.
     * @param tau Desired joint torque values.
     */
	void setJointTorques(const std::string& robot_name,
							const Eigen::VectorXd& tau);

	/**
     * @brief Set joint torque for a single joint
     * @param robot_name Name of the robot for which transaction is required.
     * @param joint_id Joint number on which to set value.
     * @param tau Value to set.
     */
	void setJointTorque(const std::string& robot_name,
						unsigned int joint_id,
						double tau);

	/**
     * @brief Read back joint torques as an array.
     * @param robot_name Name of the robot for which transaction is required.
     * @param tau_ret Array to write back joint torques into.
     */
	void getJointTorques(const std::string& robot_name,
							Eigen::VectorXd& tau_ret) const;

	/**
     * @brief Read back joint accelerations as an array.
     * @param robot_name Name of the robot for which transaction is required.
     * @param ddq_ret Array to write back joint accelerations into.
     */
	void getJointAcclerations(const std::string& robot_name,
								Eigen::VectorXd& ddq_ret) const;

	/**
     * @brief Integrate the virtual world over given time step.
     * @param timestep Time step in seconds by which to forward simulation.
     */
	void integrate(double timestep);

     /**
      * @brief      Shows the contact information, whenever a contact occurs
      */ 
     void showContactInfo(); 

     /**
      * @brief      Gets a vector of contact points and the corresponding contact forces at a gicen link of a given object. Gives everything in base frame.
      *
      * @param      contact_points  The contact points vector to be returned
      * @param      contact_forces  The contact forces vector to be returned
      * @param[in]  robot_name      The robot name
      * @param[in]  link_name       The link name
      */
     void getContactList(std::vector<Eigen::Vector3d>& contact_points, std::vector<Eigen::Vector3d>& contact_forces, 
          const::std::string& robot_name, const std::string& link_name);
public:
	/**
     * @brief Internal simulation object. For advanced users only.
     */
	SimulationInternal* _simulation_internal;
};

}

#endif //SIMULATION_INTERFACE_H
