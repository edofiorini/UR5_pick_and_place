#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <stdlib.h>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class InitialPose
{
private:
    ros::NodeHandle nh_;
    TrajClient* traj_client_;

public:
    InitialPose();
    ~InitialPose();
    void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
    control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory();
    actionlib::SimpleClientGoalState getState();
};