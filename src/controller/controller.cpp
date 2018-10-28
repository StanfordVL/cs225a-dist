#include <model/ModelInterface.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <Eigen/Dense>

#include <random>
#include <cmath>
#include <cfloat>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <signal.h>

static volatile bool runloop = true;
void stop(int) { runloop = false; }

using namespace std;

static string world_file = "";
static string robot_file = "";
static string robot_name = "";

static unsigned long long controller_counter = 0;

static std::string JOINT_TORQUES_COMMANDED_KEY = "";
static std::string JOINT_ANGLES_KEY  = "";
static std::string JOINT_VELOCITIES_KEY = "";
static std::string TIMESTAMP_KEY = "";
static std::string EE_LINK_NAME = "";

static std::string KP_POSITION_KEY = "";
static std::string KV_POSITION_KEY = "";
static std::string KP_ORIENTATION_KEY = "";
static std::string KV_ORIENTATION_KEY = "";
static std::string KP_JOINT_KEY = "";
static std::string KV_JOINT_KEY = "";
static std::string KP_JOINT_INIT_KEY = "";
static std::string KV_JOINT_INIT_KEY = "";

static std::string DESIRED_JOINT_POS = "";
static std::string DESIRED_POS_KEY_OP = "";
static std::string TARGET_POS_KEY_OP = "";
static std::string CURRENT_POS_KEY_OP = "";

void parseCommandline(int argc, char** argv)
{
  if (argc != 4) {
    cout << "Usage: project_controller <path-to-world.urdf> <path-to-robot.urdf> <robot-name>" << endl;
    exit(0);
  }
  world_file = string(argv[1]);
  robot_file = string(argv[2]);
  robot_name = string(argv[3]);
}

int main(int argc, char** argv)
{
  parseCommandline(argc, argv);
  JOINT_TORQUES_COMMANDED_KEY = "cs225a::robot::" + robot_name + "::actuators::fgc";
  JOINT_ANGLES_KEY            = "cs225a::robot::" + robot_name + "::sensors::q";
  JOINT_VELOCITIES_KEY        = "cs225a::robot::" + robot_name + "::sensors::dq";
  TIMESTAMP_KEY               = "cs225a::robot::" + robot_name + "::timestamp";
  KP_POSITION_KEY             = "cs225a::robot::" + robot_name + "::tasks::kp_pos";
  KV_POSITION_KEY             = "cs225a::robot::" + robot_name + "::tasks::kv_pos";
  KP_ORIENTATION_KEY          = "cs225a::robot::" + robot_name + "::tasks::kp_ori";
  KV_ORIENTATION_KEY          = "cs225a::robot::" + robot_name + "::tasks::kv_ori";
  KP_JOINT_KEY                = "cs225a::robot::" + robot_name + "::tasks::kp_joint";
  KV_JOINT_KEY                = "cs225a::robot::" + robot_name + "::tasks::kv_joint";
  KP_JOINT_INIT_KEY           = "cs225a::robot::" + robot_name + "::tasks::kp_joint_init";
  KV_JOINT_INIT_KEY           = "cs225a::robot::" + robot_name + "::tasks::kv_joint_init";
  DESIRED_JOINT_POS           = "cs225a::robot::" + robot_name + "::tasks::jt_pos_des";
  DESIRED_POS_KEY_OP          = "cs225a::robot::" + robot_name + "::tasks::ee_pos_des";
  TARGET_POS_KEY_OP           = "cs225a::robot::" + robot_name + "::tasks::target_pos";
  CURRENT_POS_KEY_OP          = "cs225a::robot::" + robot_name + "::tasks::ee_pos";
  EE_LINK_NAME                = robot_name + "_link_6";

  cout << "Loading URDF world model file: " << world_file << endl;
  cout << JOINT_ANGLES_KEY << endl;
  cout << JOINT_VELOCITIES_KEY << endl;

  // Make sure redis-server is running at localhost with default port 6379
  HiredisServerInfo info;
  info.hostname_ = "127.0.0.1";
  info.port_ = 6379;
  info.timeout_ = { 1, 500000 }; // 1.5 seconds
  auto redis_client = RedisClient();
  redis_client.serverIs(info);

  // Load robot
  auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);
  robot->updateModel();
  const int dof = robot->dof();
  cout << "Robot Degrees of Freedom: " << dof << endl;
  // Create a loop timer
  const double control_freq = 1000;
  LoopTimer timer;
  timer.setLoopFrequency(control_freq);   // 1 KHz
  // timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
  timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
  timer.initializeTimer(1e6); // 1 ms pause before starting loop

  /*******************************
   ***** JOINT SPACE CONTROL *****
   *******************************/

  string redis_buf; double kp_joint_init, kv_joint_init;
  Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);
  Eigen::VectorXd q_err(dof), g(dof), q_initial(dof);

  Eigen::Vector3d x_initial;
  const double TOLERANCE_Q_INIT = 0.1;
  const double TOLERANCE_DQ_INIT = 0.1;

  redis_client.getEigenMatrixDerivedString(DESIRED_JOINT_POS, q_initial);
  cout << "Joint Space Controller: Initializing to Home Position." << endl;

  while (runloop) {
    // wait for next scheduled loop (controller must run at precise rate)
    timer.waitForNextLoop();

    // read from Redis current sensor values
    redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
    redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

    // Update the model
    robot->updateModel();

    // Read in KP and KV from Redis (can be changed on the fly in Redis)
    redis_client.getCommandIs(KP_JOINT_INIT_KEY, redis_buf);
    kp_joint_init = stoi(redis_buf);
    redis_client.getCommandIs(KV_JOINT_INIT_KEY, redis_buf);
    kv_joint_init = stoi(redis_buf);

    // Break if the robot has converged to q_initial
    q_err = robot->_q - q_initial;
    if (q_err.norm() < TOLERANCE_Q_INIT && robot->_dq.norm() < TOLERANCE_DQ_INIT) {
      robot->position(x_initial, EE_LINK_NAME, Eigen::Vector3d::Zero());
      break;
    }

    // Compute torques
    robot->gravityVector(g);
    command_torques = robot->_M * (-kp_joint_init * q_err - kv_joint_init * robot->_dq); // + g;

    // Send torques
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    controller_counter++;
  }

  /*************************************
   ***** OPERATIONAL SPACE CONTROL *****
   *************************************/

  double kp_pos, kv_pos;
  double kp_ori, kv_ori;
  double kp_joint, kv_joint;

  // Initialize controller variables
  Eigen::MatrixXd lambda(6, 6);
  Eigen::MatrixXd jv(3, dof);
  Eigen::MatrixXd jw(3, dof);
  Eigen::MatrixXd j(6, dof);
  Eigen::MatrixXd N(dof, dof);

  Eigen::Vector3d target_pos;
  Eigen::Vector3d x_hat, y_hat, z_hat;
  Eigen::Vector3d current_task_vel;
  Eigen::Vector3d current_task_pos;
  Eigen::Vector3d desired_task_pos;
  Eigen::Vector3d current_angl_vel;

  Eigen::Matrix3d rotation_current;
  Eigen::Matrix3d rotation_desired;
  Eigen::Vector3d rotation_vec1;
  Eigen::Vector3d rotation_vec2;
  Eigen::Vector3d orient_error;

  Eigen::VectorXd diff_combined(6);
  Eigen::VectorXd diff_rotation(3);
  Eigen::VectorXd diff_position(3);
  Eigen::VectorXd diff_joint(dof);

  /* Compute Initial Conditions */
  desired_task_pos = x_initial;
  target_pos = x_initial;
  robot->rotation(rotation_current, EE_LINK_NAME);
  target_pos = target_pos - 0.3*rotation_current.col(2);

  /* Set Initial Conditions */
  redis_client.setEigenMatrixDerivedString(TARGET_POS_KEY_OP, target_pos);
  redis_client.setEigenMatrixDerivedString(DESIRED_POS_KEY_OP, desired_task_pos);
  cout << "Joint position initialized. Switching to operational space controller." << endl;

  // While window is open:
  while (runloop) {

    // Wait for next scheduled loop (controller must run at precise rate)
    timer.waitForNextLoop();

    // Get current simulation timestamp from Redis
    redis_client.getCommandIs(TIMESTAMP_KEY, redis_buf);
    double t_curr = stod(redis_buf);

    // Read from Redis current sensor values
    redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
    redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
    redis_client.getEigenMatrixDerivedString(TARGET_POS_KEY_OP, target_pos);
    redis_client.getEigenMatrixDerivedString(DESIRED_POS_KEY_OP, desired_task_pos);
    robot->updateModel();

    // Read in KP and KV from Redis (can be changed on the fly in Redis)
    redis_client.getCommandIs(KP_POSITION_KEY, redis_buf);
    kp_pos = stoi(redis_buf);
    redis_client.getCommandIs(KV_POSITION_KEY, redis_buf);
    kv_pos = stoi(redis_buf);
    redis_client.getCommandIs(KP_ORIENTATION_KEY, redis_buf);
    kp_ori = stoi(redis_buf);
    redis_client.getCommandIs(KV_ORIENTATION_KEY, redis_buf);
    kv_ori = stoi(redis_buf);
    redis_client.getCommandIs(KP_JOINT_KEY, redis_buf);
    kp_joint = stoi(redis_buf);
    redis_client.getCommandIs(KV_JOINT_KEY, redis_buf);
    kv_joint = stoi(redis_buf);

    /* Set Fields */
    robot->position(current_task_pos, EE_LINK_NAME, Eigen::Vector3d::Zero());
    robot->linearVelocity(current_task_vel, EE_LINK_NAME, Eigen::Vector3d::Zero());
    robot->angularVelocity(current_angl_vel, EE_LINK_NAME);
    robot->rotation(rotation_current, EE_LINK_NAME);
    robot->Jw(jw, EE_LINK_NAME);
    robot->Jv(jv, EE_LINK_NAME, Eigen::Vector3d::Zero());
    robot->nullspaceMatrix(N, jv);
    robot->gravityVector(g);

    /* Compute Desired Orientation */
    z_hat = desired_task_pos - target_pos;
    z_hat.normalize();
    x_hat << 0, -z_hat(2), z_hat(1); x_hat.normalize();
    y_hat = z_hat.cross(x_hat);

    rotation_desired.col(0) = x_hat;
    rotation_desired.col(1) = y_hat;
    rotation_desired.col(2) = z_hat;

    /* Compute Dynamics */
    diff_position = kp_pos * (desired_task_pos - current_task_pos) - kv_pos * current_task_vel;
    diff_joint = -kv_joint * robot->_dq;
    orient_error << 0, 0, 0;
    for(int i = 0; i < 3; i++) {
      rotation_vec1 << rotation_current(0, i), rotation_current(1, i), rotation_current(2, i);
      rotation_vec2 << rotation_desired(0, i), rotation_desired(1, i), rotation_desired(2, i);
      orient_error = orient_error + rotation_vec1.cross(rotation_vec2);
    }

    j << jv, jw;
    orient_error = -orient_error/2;
    robot->taskInertiaMatrixWithPseudoInv(lambda, j);
    diff_rotation  = (-kp_ori*orient_error) + (-kv_ori*current_angl_vel);
    diff_combined << diff_position, diff_rotation;

    /* Compute Command Torques */
    command_torques = j.transpose() * lambda * diff_combined + N.transpose() * robot->_M * diff_joint; // + g;
    redis_client.setEigenMatrixDerivedString(CURRENT_POS_KEY_OP, current_task_pos);
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    controller_counter++;
  }

  command_torques.setZero();
  redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
  return 0;
}
