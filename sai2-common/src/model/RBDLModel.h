/*
 * RBDLModel.h
 *
 *  Created on: Dec 14, 2016
 *      Author: Mikael Jorda
 */

#ifndef RBDLMODEL_H_
#define RBDLMODEL_H_

#include "ModelInternal.h"
#include <rbdl/Model.h>

namespace Model
{

class RBDLModel : public ModelInternal
{
public:
    RBDLModel ();
    RBDLModel (const std::string path_to_model_file, Model::ParserType parser,bool verbose);
    virtual ~RBDLModel ();


    /**
     * @brief update the dynamics. call with model joint positions, velocities and acceleration
     * @param q Joint positions
     * @param dq Joint velocities
     * @param ddq Joint accelerations
     */
    virtual void updateModel(const Eigen::VectorXd& q,
                             const Eigen::VectorXd& dq,
                             const Eigen::VectorXd& ddq);

    /**
     * @brief Gives the mass matrix of the last updated configuration
     * @param A matrix to which the mass matrix will be written. Size must be _dof x _dof
     * @param q Joint positions
     */
    virtual void massMatrix(Eigen::MatrixXd& A,
                            const Eigen::VectorXd& q);

    /**
     * @brief Gives the joint gravity torques vector of the last updated configuration
     * @param g Vector to which the joint gravity torques will be written
     * @param q Joint positions
     * @param gravity the 3d gravity vector of the world in base frame
     */
    virtual void gravityVector(Eigen::VectorXd& g,
                               const Eigen::VectorXd& q,
                               const Eigen::Vector3d& gravity = Eigen::Vector3d(0,0,-9.8));

    /**
     * @brief Gives the joint coriolis and centrifugal forces of the last updated configuration
     * @param b Vector to which the joint coriolis and centrifugal forces will be written
     * @param q Joint positions
     * @param dq Joint velocities
     */
    virtual void coriolisForce(Eigen::VectorXd& b,
                               const Eigen::VectorXd& q,
                               const Eigen::VectorXd& dq);

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
                   const Eigen::VectorXd& q);

    /**
     * @brief Full jacobian for link, relative to base (id=0) in the form [Jv; Jw]
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     * @param q Joint positions
     */
    virtual void J(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link,
                   const Eigen::VectorXd& q);


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
                    const Eigen::VectorXd& q);


    /**
     * @brief Angular velocity jacobian for link, relative to base (id=0)
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param q Joint positions
     */
    virtual void Jw(Eigen::MatrixXd& J,
                    const std::string& link_name,
                    const Eigen::VectorXd& q);



    /**
     * @brief transformation from base to link, in base coordinates.
     * @param T Transformation matrix to which the result is computed
     * @param link_name name of the link where to compute the transformation matrix
     * @param q Joint positions
     */
    virtual void transform(Eigen::Affine3d& T,
                           const std::string& link_name,
                           const Eigen::VectorXd& q);

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
                          const Eigen::VectorXd& q);

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
                          const Eigen::VectorXd& dq);

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
                              const Eigen::VectorXd& ddq);

    /**
     * @brief Rotation of a link with respect to base frame
     * @param rot Rotation matrix to which the result is written
     * @param link_name name of the link for which to compute the rotation
     * @param q Joint positions
     */
    virtual void rotation(Eigen::Matrix3d& rot,
                          const std::string& link_name,
                          const Eigen::VectorXd& q);

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
                                 const Eigen::VectorXd& dq);

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
                                     const Eigen::VectorXd& ddq);

    /**
     * @brief Gives the link id for a given nave with the right indexing for rbdl
     * @param link_name name of the link
     */
    unsigned int linkId(const std::string& link_name);


    RigidBodyDynamics::Model _rbdl_model;


};

} /* namespace Model */

#endif /* RBDLMODEL_H_ */
