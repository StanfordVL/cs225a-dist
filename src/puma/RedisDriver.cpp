/*
 * RedisDriver.cpp
 *
 *  Created on: May 4, 2017
 *      Author: Toki Migimatsu
 */

#include "RedisDriver.h"

#include <iostream>
#include <signal.h>

using namespace Puma;


/**********************
 * Driver Application *
 **********************/

static volatile bool g_runloop = true;
static RobotCom *g_puma_robot = nullptr;

static void setCtrlCHandler(void (*userCallback)(int)) {
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = userCallback;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
}

static void stop() {
	if (g_puma_robot != nullptr)
		g_puma_robot->_break();
	g_runloop = false;
	std::cout << "Sent BREAK command to Puma. Stopping run loop." << std::endl;
}
static void stop(int signal) { stop(); }

// Return true if any elements in the Eigen::MatrixXd are NaN
template<typename Derived>
static inline bool isnan(const Eigen::MatrixBase<Derived>& x) {
	return (x.array() != x.array()).any();
}

int main(int argc, char** argv)
{
	std::cout << "This program provides a Redis interface for communication with the Puma server." << std::endl;

	// Initialize driver
	RedisDriver pumaDriver;
	pumaDriver.init();
	std::cout << "Successfully connected to Puma server." << std::endl;

	// Set up ctrl-c handler
	g_puma_robot = pumaDriver.puma_robot_.get();
	setCtrlCHandler(stop);

	// Run forever
	std::cout << "Running Puma driver..." << std::endl;
	pumaDriver.run();
}


/************************************
 * RedisDriver Class Implementation *
 ************************************/

void RedisDriver::eigenVectorFromBuffer(Eigen::VectorXd& vector) {
	assert(vector.size() < SIZE_OP_SPACE_TASK);
	for (int i = 0; i < vector.size(); i++) {
		vector[i] = data_buffer_[i];
	}
}

void RedisDriver::eigenVectorToBuffer(const Eigen::VectorXd& vector) {
	assert(vector.size() < SIZE_OP_SPACE_TASK);
	for (int i = 0; i < vector.size(); i++) {
		data_buffer_[i] = vector[i];
	}
}

void RedisDriver::init() {
	// Connect to Puma server
	puma_robot_ = std::make_unique<RobotCom>();

	// Get Puma joint angles
	puma_robot_->getStatus(GET_JPOS, data_buffer_);
	eigenVectorFromBuffer(q_);

	// Initialize timer
	timer_.setLoopFrequency(kControlFreq);
	timer_.setCtrlCHandler(&stop);
	timer_.initializeTimer(kInitializationPause);

	// Connect to Redis server
	redis_client_.serverIs(kRedisServerInfo);

	// Initialize Redis keys
	redis_client_.mset({
		{KEY_CONTROL_MODE, kDefaultControlModeStr},
		{KEY_COMMAND_DATA, RedisClient::encodeEigenMatrixString(command_data_)},
		{KEY_KP, kp_str_},
		{KEY_KV, kv_str_}
	});
	redis_client_.setEigenMatrixString(KEY_JOINT_POSITIONS, q_);
	redis_client_.setEigenMatrixString(KEY_JOINT_VELOCITIES, dq_);
	redis_client_.setEigenMatrixString(KEY_JOINT_TORQUES, Gamma_);
}

void RedisDriver::run() {
	while (g_runloop) {
		// Wait for timer
		timer_.waitForNextLoop();

		// Get Redis control keys in atomic transaction
		auto values = redis_client_.mget({KEY_CONTROL_MODE, KEY_COMMAND_DATA});

		// Check for BREAK command
		std::string mode_str = values[0];
		if (mode_str == "BREAK") {
			puma_robot_->_break();
			std::cout << "BREAK" << std::endl;
			break;
		}

		// Parse control mode
		auto mode_keyval = CONTROL_MODE_MAP.find(mode_str);
		if (mode_keyval == CONTROL_MODE_MAP.end()) {
			stop();
			std::cerr << "ERROR: Redis key (" << KEY_CONTROL_MODE << ", "
			         << mode_str << ") is not a known control type." << std::endl;
			break;
		}
		control_mode_ = mode_keyval->second;

		// Filter command data
		command_data_ = RedisClient::decodeEigenMatrixString(values[1]);
		if (isnan(command_data_)) {
			stop();
			std::cerr << "ERROR: Redis key (" << KEY_COMMAND_DATA << ", "
			         << command_data_.transpose()
			         << ") contains NaN values." << std::endl;
			break;
		}

		// Check command length
		switch (control_mode_) {
			case FLOAT:
			case NJHOLD:  case JHOLD:
			case NHOLD:   case HOLD:
				command_data_ = Eigen::VectorXd::Zero(0);
				break;
			case NJMOVE:  case JMOVE:
			case NJGOTO:  case JGOTO:
			case NJTRACK: case JTRACK:
				if (command_data_.size() != DOF) {
					stop();
					std::cerr << "ERROR: Redis key (" << KEY_COMMAND_DATA << ", "
					          << command_data_.transpose() << ") must be of length "
					          << DOF << " for control mode_ " << control_mode_ << "."
					          << std::endl;
					continue;
				}
				break;
			case NXTRACK: case XTRACK:
			case NGOTO:   case GOTO:
			case NTRACK:  case TRACK:
				if (command_data_.size() != SIZE_OP_SPACE_TASK) {
					stop();
					std::cerr << "ERROR: Redis key (" << KEY_COMMAND_DATA << ", "
					          << command_data_.transpose() << ") must be of length "
					          << SIZE_OP_SPACE_TASK << " for control mode_ "
					          << control_mode_ << "." << std::endl;
					continue;
				}
				break;
			default:
				break;
		}

		// Send kp if changed
		std::string kp_str_new = redis_client_.get(KEY_KP);
		if (kp_str_new != kp_str_) {
			kp_ = RedisClient::decodeEigenMatrixString(kp_str_new);
			if (kp_.size() != DOF) {
				stop();
				std::cerr << "ERROR: Redis key (" << KEY_KP << ", "
			         	 << kp_.transpose() << ") must be of length " << DOF
			         	 << "." << std::endl;
				break;
			} else if (isnan(kp_)) {
				stop();
				std::cerr << "ERROR: Redis key (" << KEY_KP << ", "
				          << kp_.transpose()
				          << ") contains NaN values." << std::endl;
				break;
			}
			kp_str_ = kp_str_new;
			eigenVectorToBuffer(kp_);
			puma_robot_->setStatus(SET_KP, data_buffer_, kp_.size());
		}

		// Send kv if changed
		std::string kv_str_new = redis_client_.get(KEY_KV);
		if (kv_str_new != kv_str_) {
			kv_ = RedisClient::decodeEigenMatrixString(kv_str_new);
			if (kv_.size() != DOF) {
				stop();
				std::cerr << "ERROR: Redis key (" << KEY_KV << ", "
			         	 << kv_.transpose() << ") must be of length " << DOF
			         	 << "." << std::endl;
				break;
			} else if (isnan(kv_)) {
				stop();
				std::cerr << "ERROR: Redis key (" << KEY_KV << ", "
				          << kv_.transpose()
				          << ") contains NaN values." << std::endl;
				break;
			}
			kv_str_ = kv_str_new;
			eigenVectorToBuffer(kv_);
			puma_robot_->setStatus(SET_KV, data_buffer_, kv_.size());
		}

		// Send command if valid
		eigenVectorToBuffer(command_data_);
		puma_robot_->control(control_mode_, data_buffer_, command_data_.size());

		// Get Puma joint positions
		puma_robot_->getStatus(GET_JPOS, data_buffer_);
		eigenVectorFromBuffer(q_);
		redis_client_.setEigenMatrixString(KEY_JOINT_POSITIONS, q_);

		// Get Puma joint velocities
		puma_robot_->getStatus(GET_JVEL, data_buffer_);
		eigenVectorFromBuffer(dq_);
		redis_client_.setEigenMatrixString(KEY_JOINT_VELOCITIES, dq_);

		// Get Puma torques
		puma_robot_->getStatus(GET_TORQ, data_buffer_);
		eigenVectorFromBuffer(Gamma_);
		redis_client_.setEigenMatrixString(KEY_JOINT_TORQUES, Gamma_);
	}
}

