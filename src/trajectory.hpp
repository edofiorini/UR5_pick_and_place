class Trajectory
{
private:
  KDL::Tree my_tree;
  KDL::Chain chain;
public:
  RobotArm(ros::NodeHandle nh_);

  KDL::JntArray  IKinematics(double X, double Y, double Z, double roll, double pitch, double yaw,double joints[6], Eigen::MatrixXd &operational_velocities, int pos, Eigen::MatrixXd &operational_acc, int length, double vel_[6], double acc_[6], bool * flag);
};