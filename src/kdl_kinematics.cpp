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
#include <kdl/jntarray.hpp>
#include "kdl_kinematics.hpp"
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

RobotArm::RobotArm(ros::NodeHandle nh_)
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

KDL::JntArray  RobotArm::IKinematics(double X, double Y, double Z, double roll, double pitch, double yaw, double joints[6], Eigen::MatrixXd &operational_velocities, int pos, Eigen::MatrixXd &operational_acc,int length, double vel_[6], double acc_[6], bool *flag)
  {
    KDL::ChainFkSolverPos_recursive fk = KDL::ChainFkSolverPos_recursive(chain);

    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = KDL::JntArray(nj);
    std::vector< std::string > endpoints;
    endpoints.push_back("robot_arm_tool0");
 
    /*
      for the current joints positions you can read it from joint_states, remember that you have to swap the first and the third value
      joint_states publish in alphabetical order, but for the kinematics you need the actual order
    */
    for(unsigned int i=0;i<nj;i++){
        if (i == 0) {
          jointpositions(i)= joints[2];
        } else if (i == 1) {
          jointpositions(i)= joints[1];
        } else if (i == 2){
          jointpositions(i)= joints[0];
        } else{
          jointpositions(i)= joints[i];
        }
    }
   
    KDL::ChainIkSolverVel_wdls ik_v = KDL::ChainIkSolverVel_wdls(chain);
    KDL::ChainIkSolverPos_LMA	ik_p = KDL::ChainIkSolverPos_LMA(chain);
    // KDL::ChainIkSolverAcc	ik_a = KDL::ChainIkSolverAcc(chain);

    KDL::Frame cartpos;
    bool kinematics_status;
    kinematics_status = fk.JntToCart(jointpositions,cartpos); // @todo check what to do with this status

    // You have done with the initialization part, now you can use IK  
    // Use directly a quaternion or create one from RPY values
    tf::Quaternion q1;
    q1.setEuler(roll, pitch, yaw);

    //convert quaternion to rotation matrix
    tf::Matrix3x3 m_new1(q1.normalize());


    KDL::Rotation R1 = KDL::Rotation(m_new1[0][0], m_new1[0][1], m_new1[0][2], m_new1[1][0], 
      m_new1[1][1], m_new1[1][2], m_new1[2][0], m_new1[2][1], m_new1[2][2]);

    //translation
    KDL::Vector V1 = KDL::Vector(X,Y,Z);

    KDL::Frame target = KDL::Frame(R1,V1);

    KDL::JntArray target_joints = KDL::JntArray(nj);
    KDL::JntArray target_joints_vel = KDL::JntArray(nj);
    KDL::JntArray target_joints_acc = KDL::JntArray(nj);

    double result_p = ik_p.CartToJnt(jointpositions, target, target_joints); //@todo check the meaning of result -3 KDL::SolverI::E_NOERROR
    *flag = true;
    
  
    //std::cout << "\nresult ik_p "<<result_p<<std::endl;

    // tf::TransformListener listener;
    // tf::StampedTransform transform;

    // std::string *error_msg=NULL; // @todo check the transform for the velocity
      
    // listener.waitForTransform("robot_wsg50_center","robot_base_footprint", ros::Time(0),  ros::Duration(0.5), ros::Duration(0.01), error_msg=NULL); 
    // listener.lookupTransform("robot_wsg50_center","robot_base_footprint",ros::Time(0), transform);
    //listener.waitForTransform("/robot_base_link", "/robot_arm_tool0",ros::Time::now(), transform);
    // std::cout << " transform : " << transform.getOrigin().x() << " "<< transform.getOrigin().y()<< " " << transform.getOrigin().z()<< std::endl;
    // KDL::Vector v_base(0,0,0); // base del robot
    // KDL::Vector v_gripper(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z()); // end effector

    KDL::Twist tw = KDL::Twist::Zero();
    tw.vel.x(operational_velocities.coeff(0,pos));
    tw.vel.y(operational_velocities.coeff(1,pos));
    tw.vel.z(operational_velocities.coeff(2,pos));
    tw.rot.x(operational_velocities.coeff(3,pos));
    tw.rot.y(operational_velocities.coeff(4,pos));
    tw.rot.z(operational_velocities.coeff(5,pos));

    double result_v = ik_v.CartToJnt(target_joints,tw,target_joints_vel);

    if (result_p < 0 || result_v < 0) {
      *flag = false;
    }

    //std::cout<<"result ik_v " << result_v << std::endl;

    for(int idx =0;idx<6;idx++)
      vel_[idx] = target_joints_vel.data[idx];


    // KDL::Twist tw = KDL::Twist::Zero();
  //   tw.vel.x(operational_acc.coeff(0,i));
  //   tw.vel.y(operational_acc.coeff(1,i));
  //   tw.vel.z(operational_acc.coeff(2,i));
  //   tw.rot.x(operational_acc.coeff(3,i));
  //   tw.rot.y(operational_acc.coeff(4,i));
  //   tw.rot.z(operational_acc.coeff(5,i));

    // double result_a = ik_a.CartToJnt(target_joints,target_joints_vel,tw,target_joints_acc);

    // std::cout<<"result ik_a " << result_a << std::endl;

    // for(int idx =0;idx<6;idx++)
    //   acc_[idx] = target_joints_acc.data[idx];
  
    return target_joints;
  }