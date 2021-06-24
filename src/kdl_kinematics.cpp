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

using namespace KDL;



  

KDL::JntArray RobotArm::RobotArm::IKinematics(double X, double Y, double Z, double roll, double pitch, double yaw)
  {
    // verificare i valori  e sostituirli con quelli corretti passati dalla funzione
    
    std::cout << robot_desc_string << std::endl;
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
        jointpositions(i)= 0.5; //@todo check this one 
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
    q1.setEuler(roll, pitch, yaw);

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
  
    return target_joints;
  }