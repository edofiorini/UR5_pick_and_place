#ifndef JOINT_TRAJ
#define JOINT_TRAJ

#include <Eigen/Eigen>

#include "kdl_kinematics.hpp"

using namespace Eigen;
//CLASS TO BUILD JOINT TRAJECTORY
class JointPolTraj {
private:
    // Trajectory data
    int nJoints;                // Number of joints
    int samples;                // Trajectory samples
    double Ts;                  // Sampling time
    MatrixXd jointPos;          // Joints position matrix (6, samples)
    MatrixXd jointVel;          // Joints velocity matrix (6, samples)
    MatrixXd jointAcc;          // Joints acceleration matrix (6, samples)
    std::vector<double> tSeq;   // Time sequence vector

    // Type of joint trajectory
    void fifthPolTraj(double* qi, double* qf, double* dqi, double* dqf, double* d2qi, double* d2qf);

public:
    // Constructor
    JointPolTraj(MatrixXd pi, MatrixXd pf, MatrixXd PHI_i, MatrixXd PHI_f, RobotArm ra, double joints[], int nJoints, double ti, double tf, double Ts);

    // Getters
    int getNJoints();
    int getSamples();
    double getTs();
    MatrixXd getJointPos();
    MatrixXd getJointVel();
    MatrixXd getJointAcc();
    std::vector<double> getTSeq();
};

#endif
