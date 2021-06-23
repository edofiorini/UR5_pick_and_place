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
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class RobotArm
{
private:
  KDL::Tree my_tree;
  KDL::Chain chain;

public:
  RobotArm(ros::NodeHandle nh_)
  {
    std::string robot_desc_string;
    nh_.param("robot/robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree)){
        ROS_ERROR("Failed to construct kdl tree");
    }
    KDL::SegmentMap::const_iterator root_seg;
    root_seg = my_tree.getRootSegment();
    my_tree.getChain("robot_base_footprint", "robot_arm_tool0", chain);
  }

  KDL::JntArray IKinematics(double X, double Y, double Z, double roll, double pitch, double yaw)
  {
    KDL::ChainFkSolverPos_recursive fk = KDL::ChainFkSolverPos_recursive(chain);

    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    KDL::JntArray jointminpos = KDL::JntArray(nj);
    KDL::JntArray jointmaxpos = KDL::JntArray(nj);
    KDL::JntArray jointmaxvel = KDL::JntArray(nj);
    std::vector< std::string > endpoints;
    endpoints.push_back("robot_arm_tool0");

 
    /*
      for the current joints positions you can read it from joint_states, remember that you have to swap the first and the third value
      joint_states publish in alphabetical order, but for the kinematics you need the actual order
    */
    for(unsigned int i=0;i<nj;i++){
        // jointpositions(i)= CURRENT_JOINTS_POSITION; 
        jointpositions(i) = 0; // @TODO verify the current joints positions 
        jointminpos(i) = -M_PI;
        jointmaxpos(i) = M_PI;
        jointmaxvel(i) = 0.5;
    }


    KDL::ChainIkSolverVel_wdls ik_v = KDL::ChainIkSolverVel_wdls(chain);
    KDL::ChainIkSolverPos_LMA	 ik_p = KDL::ChainIkSolverPos_LMA	(chain);

    KDL::Frame cartpos;
    bool kinematics_status;
    kinematics_status = fk.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        printf("\n\n%s \n","Success, thanks KDL!");
    }else{
        printf("\n\n%s \n","Error: could not calculate forward kinematics :(");
    }


    //you have done with the initialization part, now you can use IK
    
    //Use directly a quaternion or create one from RPY values
    tf::Quaternion q1;
    q1.setRPY(roll, pitch, yaw);

    //convert quaternion to rotation matrix
    tf::Matrix3x3 m_new1(q1.normalize());

    KDL::Rotation R1 = KDL::Rotation(m_new1[0][0], m_new1[0][1], m_new1[0][2], m_new1[1][0], 
      m_new1[1][1], m_new1[1][2], m_new1[2][0], m_new1[2][1], m_new1[2][2]);

    //translation
    KDL::Vector V1 = KDL::Vector(X,Y,Z);

    KDL::Frame target = KDL::Frame(R1,V1);

    KDL::JntArray target_joints = KDL::JntArray(nj);

    double result = ik_p.CartToJnt(jointpositions, target, target_joints);

    std::cout<<"result ik_p "<<result<<std::endl;
    std::cout<<"result joints ik_p "<<target_joints.data[0]<<" "<<target_joints.data[1]<<" "
      <<target_joints.data[2]<<" "<<target_joints.data[3]<<" "<<target_joints.data[4]<<" "<<target_joints.data[5]<<std::endl;
    
    return target_joints;
  }
};