/*
 * KukaRBDLModel.h
 *
 *  Created on: March 28, 2017
 *      Author: Mikael Jorda
 */


#ifndef KUKA_RBDLMODEL_H_
#define KUKA_RBDLMODEL_H_

#include <KukaLBRDynamics/Robot.h>
#include "RBDLModel.h"
#include <Eigen/Dense>
#include <string>

namespace Model
{

/** @brief Model based upon RBDL,
 * with mass/inertia dependent functions overloaded with Kuka proprietary computation.
 *
 * @ingroup  dynamics
 */
class KukaRBDLModel : public RBDLModel 
{
public:
    KukaRBDLModel (const std::string path_to_model_file, Model::ParserType parser, bool verbose) 
    : RBDLModel(path_to_model_file, parser, verbose), _kuka_lbr_dynamics(kuka::Robot::LBRiiwa) {}
    

    virtual ~KukaRBDLModel(){}

    virtual void massMatrix(Eigen::MatrixXd& M,
                            const Eigen::VectorXd& q)
    {
        Eigen::VectorXd q_copy = q;
        _kuka_lbr_dynamics.getMassMatrix(M, q_copy);
    }


    virtual void gravityVector(Eigen::VectorXd& g,
                               const Eigen::VectorXd& q,
                               const Eigen::Vector3d& gravity = Eigen::Vector3d(0,0,-9.8))
    {
        Eigen::VectorXd q_copy = q;
        Eigen::VectorXd b = Eigen::VectorXd::Zero(_dof);
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(_dof);
        _kuka_lbr_dynamics.getCoriolisAndGravityVector(b, g, q_copy, dq);
    }


    virtual void coriolisForce(Eigen::VectorXd& b,
                               const Eigen::VectorXd& q,
                               const Eigen::VectorXd& dq)
    {
        Eigen::VectorXd q_copy = q;
        Eigen::VectorXd dq_copy = dq;
        Eigen::VectorXd g = Eigen::VectorXd::Zero(_dof);
        _kuka_lbr_dynamics.getCoriolisAndGravityVector(b, g, q_copy, dq_copy);
    }

protected:
    kuka::Robot _kuka_lbr_dynamics;

};

}

#endif /* KUKA_RBDLMODEL_H_ */
