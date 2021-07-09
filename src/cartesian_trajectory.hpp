#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace Eigen;

class CartesianTrajectory
{
private:
  void linear_tilde(MatrixXd &T, MatrixXd &p_tilde, MatrixXd &dp_tilde, MatrixXd &ddp_tilde, MatrixXd &pi, MatrixXd &pf, float ti, float tf, float Ts);
  void fifth_polinomials(MatrixXd &T, MatrixXd &q, MatrixXd &qd, MatrixXd &qdd, float ti, float tf, float qi, float dqi, float ddqi, float qf, float dqf, float ddqf, float Ts);
  void EE_orientation(MatrixXd &T, MatrixXd &PHI_i, MatrixXd &PHI_f, MatrixXd &o_tilde, MatrixXd &do_tilde, MatrixXd &ddo_tilde, MatrixXd &pi, MatrixXd &pf, float ti, float tf, float Ts);
  //all methods below are not used
  double sign_func(double x);
  double vecangle(Vector3d &v1, Vector3d &v2, Vector3d &normal);
  int circular_length(MatrixXd &pi, MatrixXd &pf, float Ts, MatrixXd &c);
  void circular_motion(MatrixXd &T, MatrixXd &p, MatrixXd &dp, MatrixXd &ddp, MatrixXd &pi, MatrixXd &pf, float Ts, MatrixXd &c, int length);
  void linear_motion(MatrixXd &T, MatrixXd &p, MatrixXd &dp, MatrixXd &ddp, MatrixXd &s, MatrixXd &pi, MatrixXd &pf, float Ts, int length1);
  void circular_tilde(MatrixXd &T, MatrixXd &p_tilde, MatrixXd &dp_tilde, MatrixXd &ddp_tilde, MatrixXd &pi, MatrixXd &pf, MatrixXd &c, float ti, float tf, float Ts, int length_qf);
  void frenet_frame(MatrixXd &p, MatrixXd &dp, MatrixXd &ddp, MatrixXd &o_EE_t, MatrixXd &o_EE_n, MatrixXd &o_EE_b, MatrixXd &PHI_i, MatrixXd &PHI_f, int length);

public:
  // Properties
  // Declaration of data matrices
  MatrixXd dataPosition; // (6, length);
  MatrixXd dataVelocities; // (6, length);
  MatrixXd dataAcceleration; // (6, length);
  int length;

  CartesianTrajectory(MatrixXd pi, MatrixXd pf, MatrixXd PHI_i, MatrixXd PHI_f,float ti, float tf, float Ts);

  float get_length();
};