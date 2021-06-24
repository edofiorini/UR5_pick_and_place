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
class RobotArm
{
  private:
    KDL::Tree my_tree;
    KDL::Chain chain;
    std::string robot_desc_string;
  public:
    RobotArm(ros::NodeHandle nh_)
  {
    
    nh_.param("robot/robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
    }
    KDL::SegmentMap::const_iterator root_seg;
    root_seg = my_tree.getRootSegment();
    my_tree.getChain("robot_base_footprint", "robot_arm_tool0", chain);  
  }

    KDL::JntArray IKinematics(double X, double Y, double Z, double roll, double pitch, double yaw);
};