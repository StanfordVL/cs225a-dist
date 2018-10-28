/*
 * SimulationInterface.cpp
 *
 *  Created on: Dec 27, 2016
 *      Author: Shameek Ganguly
 */

#include "SimulationInterface.h"
#include "SimulationInternal.h"
#include "Sai2Simulation.h"

namespace Simulation {

SimulationInterface::SimulationInterface(const std::string& path_to_world_file,
						            		Simulation::SimulationType simulation_type,
						            		Simulation::ParserType parser,
						            		bool verbose)
: _simulation_internal(NULL) {
	// initialize internal model depending on what type of model and parser are
	// chosen by user
	switch (simulation_type) {
		case sai2simulation:
			_simulation_internal = new Sai2Simulation(
					path_to_world_file,
            		parser,
            		verbose);
			break;
	}
}

SimulationInterface::~SimulationInterface() {
	delete _simulation_internal;
	_simulation_internal = NULL;
}

// read degrees of freedom
unsigned int SimulationInterface::dof(const std::string& robot_name) const {
	return _simulation_internal->dof(robot_name);
}

// set joint positions
void SimulationInterface::setJointPositions(const std::string& robot_name,
											const Eigen::VectorXd& q) {
	_simulation_internal->setJointPositions(robot_name, q);
}

// read joint positions
void SimulationInterface::getJointPositions(const std::string& robot_name,
											Eigen::VectorXd& q_ret) const {
	_simulation_internal->getJointPositions(robot_name, q_ret);
}

// set joint position for a single joint
void SimulationInterface::setJointPosition(const std::string& robot_name,
											unsigned int joint_id,
											double position) {
	_simulation_internal->setJointPosition(robot_name, joint_id, position);
}

// set joint velocities
void SimulationInterface::setJointVelocities(const std::string& robot_name,
												const Eigen::VectorXd& dq) {
	_simulation_internal->setJointVelocities(robot_name, dq);
}

// set joint velocity for a single joint
void SimulationInterface::setJointVelocity(const std::string& robot_name,
											unsigned int joint_id,
											double velocity) {
	_simulation_internal->setJointVelocity(robot_name, joint_id, velocity);
}

// read joint velocities
void SimulationInterface::getJointVelocities(const std::string& robot_name,
												Eigen::VectorXd& dq_ret) const {
	_simulation_internal->getJointVelocities(robot_name, dq_ret);
}

// set joint torques
void SimulationInterface::setJointTorques(const std::string& robot_name,
											const Eigen::VectorXd& tau) {
	_simulation_internal->setJointTorques(robot_name, tau);
}

// set joint torque for a single joint
void SimulationInterface::setJointTorque(const std::string& robot_name,
											unsigned int joint_id,
											double tau) {
	_simulation_internal->setJointTorque(robot_name, joint_id, tau);
}

// read joint torques
void SimulationInterface::getJointTorques(const std::string& robot_name,
											Eigen::VectorXd& tau_ret) const {
	_simulation_internal->getJointTorques(robot_name, tau_ret);
}

// read joint accelerations
void SimulationInterface::getJointAcclerations(const std::string& robot_name,
												Eigen::VectorXd& ddq_ret) const {
	_simulation_internal->getJointAcclerations(robot_name, ddq_ret);
}

// integrate ahead
void SimulationInterface::integrate(double timestep) {
	_simulation_internal->integrate(timestep);	
}

void SimulationInterface::showContactInfo() {
	_simulation_internal->showContactInfo();
}

void SimulationInterface::getContactList(std::vector<Eigen::Vector3d>& contact_points, std::vector<Eigen::Vector3d>& contact_forces, 
     const::std::string& robot_name, const std::string& link_name) {
	_simulation_internal->getContactList(contact_points, contact_forces, robot_name, link_name);
}

}
