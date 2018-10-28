/*
 * ModelInternal.h
 *
 *  Created on: Dec 13, 2016
 *      Author: Mikael Jorda
 */

#ifndef MODELINTERNAL_H_
#define MODELINTERNAL_H_

#include <Eigen/Dense>

#include <vector>
#include <map>
#include <string>
#include <iostream>

#include "ModelInterface.h"

namespace Model
{
class ModelInternal
{
protected:
    ModelInternal(){;}

public:


    /**
     * @brief update the dynamics. call with model joint positions, velocities and acceleration
     * @param q Joint positions
     * @param dq Joint velocities
     * @param ddq Joint accelerations
     */
    virtual void updateModel(const Eigen::VectorXd& q,
                             const Eigen::VectorXd& dq,
                             const Eigen::VectorXd& ddq) = 0;

    /**
     * @brief Gives the mass matrix
     * @param A matrix to which the mass matrix will be written. Size must be _dof x _dof
     * @param q Joint positions
     */
    virtual void massMatrix(Eigen::MatrixXd& A,
                            const Eigen::VectorXd& q) = 0;

    /**
     * @brief Gives the joint gravity torques vector
     * @param g Vector to which the joint gravity torques will be written
     * @param q Joint positions
     * @param gravity the 3d gravity vector of the world in base frame
     */
    virtual void gravityVector(Eigen::VectorXd& g,
                               const Eigen::VectorXd& q,
                               const Eigen::Vector3d& gravity = Eigen::Vector3d(0,0,-9.8)) = 0;

    /**
     * @brief Gives the joint coriolis and centrifugal forces
     * @param b Vector to which the joint coriolis and centrifugal forces will be written
     * @param q Joint positions
     * @param dq Joint velocities
     */
    virtual void coriolisForce(Eigen::VectorXd& b,
                               const Eigen::VectorXd& q,
                               const Eigen::VectorXd& dq) = 0;

    /**
     * @brief Full jacobian for link, relative to base (id=0) in the form [Jv; Jw]
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     * @param q Joint positions
     */
    virtual void J_0(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link,
                   const Eigen::VectorXd& q) = 0;

    /**
     * @brief Full jacobian for link, relative to base (id=0) in the form [Jw; Jv]
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     * @param q Joint positions
     */
    virtual void J(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link,
                   const Eigen::VectorXd& q) = 0;


    /**
     * @brief Velocity jacobian for point on link, relative to base (id=0)
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     * @param q Joint positions
     */
    virtual void Jv(Eigen::MatrixXd& J,
                    const std::string& link_name,
                    const Eigen::Vector3d& pos_in_link,
                    const Eigen::VectorXd& q) = 0;


    /**
     * @brief Angular velocity jacobian for link, relative to base (id=0)
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param q Joint positions
     */
    virtual void Jw(Eigen::MatrixXd& J,
                    const std::string& link_name,
                    const Eigen::VectorXd& q) = 0;



    /**
     * @brief transformation from base to link, in base coordinates.
     * @param T Transformation matrix to which the result is computed
     * @param link_name name of the link where to compute the transformation matrix
     * @param q Joint positions
     */
    virtual void transform(Eigen::Affine3d& T,
                           const std::string& link_name,
                           const Eigen::VectorXd& q) = 0;

    /**
     * @brief Position from base to point in link, in base coordinates
     * @param pos Vector of position to which the result is written
     * @param link_name name of the link in which is the point where to compute the position
     * @param pos_in_link the position of the point in the link, in local link frame
     * @param q Joint positions
     */
    virtual void position(Eigen::Vector3d& pos,
                          const std::string& link_name,
                          const Eigen::Vector3d& pos_in_link,
                          const Eigen::VectorXd& q) = 0;

    /**
     * @brief Velocity of point in link, in base coordinates
     * @param vel Vector of velocities to which the result is written
     * @param link_name name of the link in which is the point where to compute the velocity
     * @param pos_in_link the position of the point in the link, in local link frame
     * @param q Joint positions
     * @param dq Joint velocities
     */
    virtual void linearVelocity(Eigen::Vector3d& vel,
                          const std::string& link_name,
                          const Eigen::Vector3d& pos_in_link,
                          const Eigen::VectorXd& q,
                          const Eigen::VectorXd& dq) = 0;

    /**
     * @brief Acceleration of point in link, in base coordinates
     * @param accel Vector of accelerations to which the result is written
     * @param link_name name of the link in which is the point where to compute the acceleration
     * @param pos_in_link the position of the point in the link, in local link frame
     * @param q Joint positions
     * @param dq Joint velocities
     * @param ddq Joint accelerations
     */
    virtual void linearAcceleration(Eigen::Vector3d& accel,
                              const std::string& link_name,
                              const Eigen::Vector3d& pos_in_link,
                              const Eigen::VectorXd& q,
                              const Eigen::VectorXd& dq,
                              const Eigen::VectorXd& ddq) = 0;

    /**
     * @brief Rotation of a link with respect to base frame
     * @param rot Rotation matrix to which the result is written
     * @param link_name name of the link for which to compute the rotation
     * @param q Joint positions
     */
    virtual void rotation(Eigen::Matrix3d& rot,
                          const std::string& link_name,
                          const Eigen::VectorXd& q) = 0;

    /**
     * @brief Angular velocity of a link with respect to base frame
     * @param avel Vector to which the result is written
     * @param link_name name of the link for which to compute the angular velocity
     * @param q Joint positions
     * @param dq Joint velocities
     */
    virtual void angularVelocity(Eigen::Vector3d& avel,
                                 const std::string& link_name,
                                 const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& dq) = 0;

    /**
     * @brief Angular acceleration of a link with respect to base frame
     * @param aaccel Vector to which the result is written
     * @param link_name name of the link for which to compute the angular acceleration
     * @param q Joint positions
     * @param dq Joint velocities
     * @param ddq Joint accelerations
     */
    virtual void angularAcceleration(Eigen::Vector3d& aaccel,
                                     const std::string& link_name,
                                     const Eigen::VectorXd& q,
                                     const Eigen::VectorXd& dq,
                                     const Eigen::VectorXd& ddq) = 0;


    /// \brief Degrees of freedom of the robot
    int _dof;

};

} /* namespace Model */

#endif /* MODELINTERNAL_H_ */
