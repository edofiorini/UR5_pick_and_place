#include "joint_pol_traj.hpp"

// Constructor
JointPolTraj::JointPolTraj(MatrixXd pi, MatrixXd pf, MatrixXd PHI_i, MatrixXd PHI_f, RobotArm ra, double joints[], int nJoints, double ti, double tf, double Ts) {

    this->nJoints = nJoints;
    this->Ts = Ts;

    // Number of samples must take into account also the tf sample,
    // thus it is equal to floor() + 1
    this->samples = 1 + (int) floor((tf - ti) / Ts);

    this->jointPos = MatrixXd(nJoints, samples);
    this->jointVel = MatrixXd(nJoints, samples);
    this->jointAcc = MatrixXd(nJoints, samples);

    // Initialize time sequence vector
    for (int i = 0; i < samples-1; i++) {
        this->tSeq.push_back(ti + Ts*i);
    }
    this->tSeq.push_back(tf);

    // Inverse kinematics to compute initial and final condition
    KDL::JntArray _qi, _qf;
    double qi[nJoints], qf[nJoints], dqi[nJoints], dqf[nJoints], d2qi[nJoints], d2qf[nJoints];

    MatrixXd opVel(nJoints, 1), opAcc(nJoints, 1);
    opVel << 0, 0, 0, 0, 0, 0;
    opAcc << 0, 0, 0, 0, 0, 0;

    // Initial joints configuration, velocity and acceleration
    _qi = ra.IKinematics(
        pi(0, 0), pi(1, 0), pi(2, 0),
        PHI_i(0, 0), PHI_i(1, 0), PHI_i(2, 0),
        joints,
        opVel, 0, opAcc, 0,
        dqi, d2qi);

    // Final joints configuration, velocity and acceleration
    _qf = ra.IKinematics(
        pf(0, 0), pf(1, 0), pf(2, 0),
        PHI_f(0, 0), PHI_f(1, 0), PHI_f(2, 0),
        joints,
        opVel, 0, opAcc, 0,
        dqf, d2qf);

    for (int j = 0; j<nJoints; j++) {
        qi[j] = _qi.data[j];
        qf[j] = _qf.data[j];
    }

    // Compute quintic polynomial trajectory for each joint
    this->fifthPolTraj(qi, qf, dqi, dqf, d2qi, d2qf);
}

// Type of trajectory
void JointPolTraj::fifthPolTraj(double* qi, double* qf, double* dqi, double* dqf, double* d2qi, double* d2qf) {

    double ti = tSeq.at(0);
    double tf = tSeq.at(samples-1);
    double deltaT = tf-ti;

    // Matrix H is the same for each joint
    MatrixXd H(6, 6);
    H << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0,
        1, deltaT, pow(deltaT, 2), pow(deltaT, 3), pow(deltaT, 4), pow(deltaT, 5),
        0, 1, 2 * deltaT, 3 * pow(deltaT, 2), 4 * pow(deltaT, 3), 5 * pow(deltaT, 4),
        0, 0, 2, 6 * deltaT, 12 * pow(deltaT, 2), 20 * pow(deltaT, 3);

    // Compute the polynomials by taking into account initial and final condition of each joint
    for (int j = 0; j<nJoints; j++) {
        MatrixXd Q(6, 1);
        Q << qi[j], dqi[j], d2qi[j], qf[j], dqf[j], d2qf[j];

        // Coefficients of j-th polynomial
        MatrixXd a = H.inverse() * (Q);
        double a0 = a.row(0).coeff(0, 0);
        double a1 = a.row(1).coeff(0, 0);
        double a2 = a.row(2).coeff(0, 0);
        double a3 = a.row(3).coeff(0, 0);
        double a4 = a.row(4).coeff(0, 0);
        double a5 = a.row(5).coeff(0, 0);

        // Compute j-th trajectory
        for (int i = 0; i < samples; i++) {
            double t = i*Ts;
            this->jointPos(j, i) = a5*pow(t, 5) + a4*pow(t, 4) + a3*pow(t, 3) + a2*pow(t, 2) + a1*t + a0;
            this->jointVel(j, i) = 5*a5*pow(t, 4) + 4*a4*pow(t, 3) + 3 * a3*pow(t, 2) + 2*a2*t + a1;
            this->jointAcc(j, i) = 20*a5*pow(t, 3) + 12*a4*pow(t, 2) + 6*a3*t + 2*a2;
        }
    }
}

// GETTERS

int JointPolTraj::getNJoints() { return nJoints; }
int JointPolTraj::getSamples() { return samples; }
double JointPolTraj::getTs() { return Ts; }
MatrixXd JointPolTraj::getJointPos() { return jointPos; }
MatrixXd JointPolTraj::getJointVel() { return jointVel; }
MatrixXd JointPolTraj::getJointAcc() { return jointAcc; }
std::vector<double> JointPolTraj::getTSeq() { return tSeq; }
