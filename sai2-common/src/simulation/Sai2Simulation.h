/*
 * Sai2Simulation.h
 *
 *  Created on: Dec 27, 2016
 *      Author: Shameek Ganguly
 */

#ifndef SAI2_SIMULATION_H
#define SAI2_SIMULATION_H

#include "SimulationInterface.h"
#include "SimulationInternal.h"
#include <Eigen/Geometry>

// forward define from sai2-simulation
class cDynamicWorld;

namespace Simulation {

class Sai2Simulation: public SimulationInternal {
public:
	/**
     * @brief Creates a simulation interface to a Sai2-Simulation engine. Supports time integration, constraint-based contact and collision resolution.
     * @param path_to_world_file A path to the file containing the model of the virtual world (urdf and yml files supported).
     * @param parser The type of parser to use in function of the file type (urdf or yml).
     * @param verbose To display information about the robot model creation in the terminal or not.
     */
	Sai2Simulation(const std::string& path_to_world_file,
		            		Simulation::ParserType parser,
		            		bool verbose);

	// \brief Destructor to clean up internal Sai2-Simulation model
	~Sai2Simulation();

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
     * @brief Read back joint torques as an array. NOTE: Currently unsupported due to lack of support in Sai2-Simulation.
     * @param robot_name Name of the robot for which transaction is required.
     * @param tau_ret Array to write back joint torques into.
     */
	void getJointTorques(const std::string& robot_name,
							Eigen::VectorXd& tau_ret) const;

	/**
     * @brief Read back joint accelerations as an array. NOTE: Currently unsupported due to lack of support in Sai2-Simulation
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

     /* Sai2-Simulation specific interface */

     /**
      * @brief      Set the co-efficient of kinematic restitution: for all objects
      * @param      restitution  Value to set
      */
     void setCollisionRestitution(double restitution);

     /**
      * @brief      Set the co-efficient of kinematic restitution: for a named link on a named robot
      * @param      robot_name  Robot on which to set the value
      * @param      link_name  Robot on which to set the value
      * @param      restitution  Value to set
      */
     void setCollisionRestitution(const std::string& robot_name,
                                        const std::string& link_name,
                                        double restitution);

     /**
      * @brief      Get the co-efficient of kinematic restitution: for a named link on a named robot
      * @param      robot_name  Robot from which to get the value
      * @param      link_name  Robot from which to get the value
      * @return     Current value
      */
     double getCollisionRestitution(const std::string& robot_name,
                                        const std::string& link_name) const;

     /**
      * @brief      Set the co-efficient of static friction: for all objects
      * @param      restitution  Value to set
      */
     void setCoeffFrictionStatic(double static_friction); // for all objects

     /**
      * @brief      Set the co-efficient of kinematic restitution: for a named link on a named robot
      * @param      robot_name  Robot on which to set the value
      * @param      link_name  Robot on which to set the value
      * @param      restitution  Value to set
      */
     void setCoeffFrictionStatic(const std::string& robot_name,
                                   const std::string& link_name,
                                   double static_friction);

     /**
      * @brief      Get the co-efficient of static friction: for a named link on a named robot
      * @param      robot_name  Robot from which to get the value
      * @param      link_name  Robot from which to get the value
      * @return     Current value
      */
     double getCoeffFrictionStatic(const std::string& robot_name,
                                   const std::string& link_name) const;

     /**
      * @brief      Set the co-efficient of dynamic friction: for all objects
      * @param      restitution  Value to set
      */
     void setCoeffFrictionDynamic(double dynamic_friction); // for all objects

     /**
      * @brief      Set the co-efficient of kinematic restitution: for a named link on a named robot
      *
      * @param      robot_name  Robot on which to set the value
      * @param      link_name  Robot on which to set the value
      * @param      restitution  Value to set
      */
     void setCoeffFrictionDynamic(const std::string& robot_name,
                                   const std::string& link_name,
                                   double dynamic_friction);

     /**
      * @brief      Get the co-efficient of dynamic friction: for a named link on a named robot
      * @param      robot_name  Robot from which to get the value
      * @param      link_name  Robot from which to get the value
      * @return     Current value
      */
     double getCoeffFrictionDynamic(const std::string& robot_name,
                                        const std::string& link_name) const;

     /**
      * @brief      Get affine transform from the global frame to the base frame of a named robot
      * @param      robot_name  Robot from which to get the value
      * @return     Transform
      */
     Eigen::Affine3d getRobotBaseTransform(const std::string& robot_name) const;

public:
	/**
     * @brief Internal dynamics world object.
     */
	cDynamicWorld* _world;
};

}

#endif //SAI2_SIMULATION_H
