#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <stdlib.h>
#include "robot_arm2.hpp"

InitialPose::InitialPose()
{
  // tell the action client that we want to spin a thread by default
  traj_client_ = new TrajClient("robot/arm/pos_traj_controller/follow_joint_trajectory", true);


  // wait for action server to come up
  while(!traj_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the joint_trajectory_action server");
  }
  std::cout<<"Done loading"<<std::endl;

}

InitialPose::~InitialPose()
{
  delete traj_client_;
}

//! Sends the command to start a given trajectory
void InitialPose::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
{
  // When to start the trajectory: 1s from now
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  traj_client_->sendGoal(goal);
}


//! Generates a simple trajectory with two waypoints, used as an example
/*! Note that this trajectory contains two waypoints, joined together
    as a single trajectory. Alternatively, each of these waypoints could
    be in its own trajectory - a trajectory can have one or more waypoints
    depending on the desired application.
*/
control_msgs::FollowJointTrajectoryGoal InitialPose::armExtensionTrajectory()
{
  //our goal variable
  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.header.seq = 1;
  goal.trajectory.header.stamp = ros::Time::now();;
  goal.trajectory.header.frame_id = "robot_base_footprint";

  // First, the joint names, which apply to all waypoints
  goal.trajectory.joint_names.clear();
  goal.trajectory.joint_names.push_back("robot_arm_shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("robot_arm_shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("robot_arm_elbow_joint");
  goal.trajectory.joint_names.push_back("robot_arm_wrist_1_joint");
  goal.trajectory.joint_names.push_back("robot_arm_wrist_2_joint");
  goal.trajectory.joint_names.push_back("robot_arm_wrist_3_joint");

  // We will have two waypoints in this goal trajectory
  goal.trajectory.points.resize(1);

  // First trajectory point
  // Positions
  int ind = 0;

  goal.trajectory.points[ind].positions.resize(6);

  goal.trajectory.points[ind].positions[0] = 0.41;
  goal.trajectory.points[ind].positions[1] = -1.07;
  goal.trajectory.points[ind].positions[2] = -1.59;
  goal.trajectory.points[ind].positions[3] = -1.46;
  goal.trajectory.points[ind].positions[4] = 1.26;
  goal.trajectory.points[ind].positions[5] = -1.19;

  goal.trajectory.points[ind].velocities.resize(6);
  for (size_t j = 0; j < 6; ++j)
  {
    goal.trajectory.points[ind].velocities[j] = 0.0;
  }
  // To be reached 2 seconds after starting along the trajectory
  goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);
  std::cout<<"Done traj"<<std::endl;
  //we are done; return the goal
  return goal;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState InitialPose::getState()
{
  return traj_client_->getState();
}