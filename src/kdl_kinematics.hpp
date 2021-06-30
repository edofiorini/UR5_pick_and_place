#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <kdl/treeiksolverpos_online.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

class RobotArm
{
private:
  KDL::Tree my_tree;
  KDL::Chain chain;
public:
  RobotArm(ros::NodeHandle nh_);

  KDL::JntArray  IKinematics(double X, double Y, double Z, double roll, double pitch, double yaw,double joints[6], Eigen::MatrixXd &operational_velocities, int pos, Eigen::MatrixXd &operational_acc, int length, double vel_[6], double acc_[6]);
};