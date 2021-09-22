#include <iostream>
#include <sstream>
#define _USE_MATH_DEFINES // For PI costants
#include <cmath>
#include <complex>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>
#include "ros/ros.h"
#include <ros/package.h>
#include "ros/duration.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#include "robot_arm2.hpp"
#include "kdl_kinematics.hpp"
#include "cartesian_trajectory.hpp"
#include "joint_pol_traj.hpp"
#include "aruco_info.hpp"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// PI costants
#define PI M_PI    // pi
#define PI2 M_PI_2 // pi/2

using namespace Eigen;

bool joints_done = false;

// Message data
cv_bridge::CvImagePtr imagePtr;
cv::Mat K(3, 3, CV_64F);
cv::Mat D(1, 5, CV_64F);

double joints[6];

// Topic
ros::Publisher dataPub;
ros::Subscriber imageSub, cameraSub, joint_state_sub;
tf2_ros::Buffer tfBuffer;

//for trajectory server 
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;
arm_control_client_Ptr ArmClient;

void createArmClient(arm_control_client_Ptr& actionClient)
{
  ROS_INFO("Creating action client to arm controller ...");

  actionClient.reset( new arm_control_client("/robot/arm/pos_traj_controller/follow_joint_trajectory") );

  int iterations = 0, max_iterations = 3;
  // Wait for arm controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the arm_controller_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}




void cameraCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    // Retrive camera matrix
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            K.at<double>(i, j) = msg->K[i * 3 + j];
}

void jointsCallback(const sensor_msgs::JointState &msg)
{
    // std::cout << msg.position[0] << std::endl;
    for (int i = 0; i < 6; i++)
    {
        joints[i] = msg.position[i];
        //std::cout<<"giunto "<<joints[i]<<std::endl;
    }
    joints_done = true;
}

void addToTf(int id, cv::Vec3d rvec, cv::Vec3d tvec)
{
    //robot_wrist_rgbd_color_frame oppure robot_wrist_rgbd_color_optical_frame
    //@todo capire se è visibile sto frame da qualche parte
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "robot_wrist_rgbd_color_optical_frame";
    transformStamped.child_frame_id = "aruco_" + std::to_string(id);
    //ROS_INFO("created frame: " +  transformStamped.child_frame_id);
    //std::cout << "created frame: " << transformStamped.child_frame_id << std::endl;
    transformStamped.transform.translation.x = -tvec[0]; //rotated aruco
    transformStamped.transform.translation.y = tvec[1];
    transformStamped.transform.translation.z = tvec[2];
    tf2::Quaternion q;
    q.setRPY(rvec[0], rvec[1], rvec[2]);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    ros::spinOnce();
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{

    cv::Mat image, imageCopy;
    try
    {
        // Retrive image from camera topic
        imagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image = imagePtr->image;
    cv::flip(image, image, 1);
    image.copyTo(imageCopy);

    // Aruco detection
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejCandidates);

    // No aruco detected
    if (ids.size() <= 0)
        return;

    // At least one marker has been detected
    //ROS_INFO("Cubes detected!");
    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

    // Aruco information container
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, 0.03, K, D, rvecs, tvecs);
    // draw axis for each marker
    for (int i = 0; i < ids.size(); i++)
    {
        // Save aruco id, rotation and traslation
        addToTf(ids[i], rvecs[i], tvecs[i]);
        cv::aruco::drawAxis(imageCopy, K, D, rvecs[i], tvecs[i], 0.1);
    }

    // Show results
    //cv::imshow("out", imageCopy);

    //char key = (char) cv::waitKey(1);
}

geometry_msgs::TransformStamped getArucoTransformStamped(int id)
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("robot_base_footprint", "aruco_" + std::to_string(id), ros::Time(0));
    }
    catch (tf::TransformException &ex)
    {

        ros::Duration(1.0).sleep();
        ROS_ERROR("%s", ex.what());
    }
    return transformStamped;
}

void sendTrajectory(MatrixXd pi, MatrixXd pf, MatrixXd PHI_i, MatrixXd PHI_f, double ti, double tf, double Ts, RobotArm ra)
{
    std::cout << "Initializing operational space trajectory..." << std::endl;
    CartesianTrajectory *trajectory;
    trajectory = new CartesianTrajectory(pi, pf, PHI_i, PHI_f, ti, tf, Ts);
    std::cout << "Trajectory initialized!" << std::endl;

    int length = trajectory->get_length();
    double previous_vel_[6];
    KDL::JntArray target_joints;
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {
        "robot_arm_shoulder_pan_joint",
        "robot_arm_shoulder_lift_joint",
        "robot_arm_elbow_joint",
        "robot_arm_wrist_1_joint",
        "robot_arm_wrist_2_joint",
        "robot_arm_wrist_3_joint"};

    for (int i = 0; i < length; i++)
    {
        double vel_[6];
        double acc_[6];

        std::cout
            << "\npoints "
            << trajectory->dataPosition.coeff(0, i)
            << " "
            << trajectory->dataPosition.coeff(1, i)
            << " "
            << trajectory->dataPosition.coeff(2, i)
            << " "
            << trajectory->dataPosition.coeff(3, i)
            << " "
            << trajectory->dataPosition.coeff(4, i)
            << " "
            << trajectory->dataPosition.coeff(5, i)
            << std::endl;

        target_joints = ra.IKinematics(
            trajectory->dataPosition.coeff(0, i),
            trajectory->dataPosition.coeff(1, i),
            trajectory->dataPosition.coeff(2, i),
            trajectory->dataPosition.coeff(3, i), // @todo inspect the use of nan in orientation
            trajectory->dataPosition.coeff(4, i),
            trajectory->dataPosition.coeff(5, i),
            joints,
            trajectory->dataVelocities,
            i,
            trajectory->dataAcceleration,
            length,
            vel_,
            acc_);

        bool check = true;
        for (int j = 0; j < 6; j++)
        {
            if (target_joints.data[j] > 3.14 || target_joints.data[j] < -3.14)
            {
                check = false;
                break;
            }
        }

        if (check)
        {
            //if (i == 0 || i == round(length/2) || i == length -1) {
            std::cout << "Sending data to Ros" << std::endl;

            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.resize(6);
            point.positions = {
                target_joints.data[0],
                target_joints.data[1],
                target_joints.data[2],
                target_joints.data[3],
                target_joints.data[4],
                target_joints.data[5]};
            /*point.velocities.resize(6);
      point.velocities = {vel_[0],vel_[1],vel_[2],vel_[3],vel_[4],vel_[5]};
      point.accelerations.resize(6);
      point.accelerations = {
        (vel_[0]-previous_vel_[0])/Ts,
        (vel_[1]-previous_vel_[1])/Ts,
        (vel_[2]-previous_vel_[2])/Ts,
        (vel_[3]-previous_vel_[3])/Ts,
        (vel_[4]-previous_vel_[4])/Ts,
        (vel_[5]-previous_vel_[5])/Ts
      };*/
            point.time_from_start = ros::Duration(i * Ts);
            points.push_back(point);
        }

        for (int j = 0; j < 6; j++)
            previous_vel_[j] = vel_[j];
    }

    goal.trajectory.points = points;

    ArmClient->sendGoalAndWait(goal);
}

void sendJointTraj(MatrixXd pi, MatrixXd pf, MatrixXd PHI_i, MatrixXd PHI_f, double ti, double tf, double Ts, RobotArm ra)
{
    std::cout << "Initializing joint space trajectory..." << std::endl;
    JointPolTraj *trajectory;
    trajectory = new JointPolTraj(pi, pf, PHI_i, PHI_f, ra, joints, 6, ti, tf, Ts);
    MatrixXd jointPos = trajectory->getJointPos();
    MatrixXd jointVel = trajectory->getJointVel();

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {
        "robot_arm_shoulder_pan_joint",
        "robot_arm_shoulder_lift_joint",
        "robot_arm_elbow_joint",
        "robot_arm_wrist_1_joint",
        "robot_arm_wrist_2_joint",
        "robot_arm_wrist_3_joint"};

    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    for (int i = 0; i < trajectory->getSamples(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint point;

        point.positions.resize(6);
        point.time_from_start = ros::Duration(i * Ts);
        point.positions = {
            jointPos.coeff(0, i),
            jointPos.coeff(1, i),
            jointPos.coeff(2, i),
            jointPos.coeff(3, i),
            jointPos.coeff(4, i),
            jointPos.coeff(5, i)};
        // point.velocities.resize(6);
        // point.velocities = {
        //     jointVel.coeff(0, i),
        //     jointVel.coeff(1, i),
        //     jointVel.coeff(2, i),
        //     jointVel.coeff(3, i),
        //     jointVel.coeff(4, i),
        //     jointVel.coeff(5, i)};

        points.push_back(point);
    }

    goal.trajectory.points = points;
    ArmClient->sendGoalAndWait(goal);
}

control_msgs::FollowJointTrajectoryGoal getVerticalGoal()
{
    control_msgs::FollowJointTrajectoryGoal goal;
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;

    goal.trajectory.joint_names = {
        "robot_arm_shoulder_pan_joint",
        "robot_arm_shoulder_lift_joint",
        "robot_arm_elbow_joint",
        "robot_arm_wrist_1_joint",
        "robot_arm_wrist_2_joint",
        "robot_arm_wrist_3_joint"};

    double pos[]{0, -0.8, -PI2};

    trajectory_msgs::JointTrajectoryPoint point;
    for (int i = 0; i < 3; i++)
    {
        point.positions.resize(6);
        point.positions = {0, pos[i], 0, 0, 0, 0};
        point.time_from_start = ros::Duration(i);
        points.push_back(point);
    }

    goal.trajectory.points = points;
    return goal;
    
}

void pickAndPlaceSingleObject(
    int arucoId, MatrixXd detectionP, MatrixXd detectionPHI, double marginX, double marginY, double marginZ, MatrixXd finalPHI,
    ros::Rate loop_rate, RobotArm ra, double Ts, bool useJointTraj, int startTime)
{
   
    //Support matrices for trajectory computation
    MatrixXd pf(3, 1);
    MatrixXd PHI_f(3, 1);
    MatrixXd pi(3, 1);
    MatrixXd PHI_i(3, 1);
    MatrixXd p_home(3, 1);
    MatrixXd PHI_home(3, 1);

    p_home << 0.077, -0.161, 1.123;

    PHI_home << 0, -PI, -PI2;

    double ti = 0.0;
    double tf = 4.0;
    double alpha, beta, gamma;

    //Moving to home
    std::cout << "Moving to home... " << std::endl;
    KDL::Frame fr = ra.FKinematics(joints);
    pi << fr.p.x(), fr.p.y(), fr.p.z();
    fr.M.GetRPY(alpha, beta, gamma);
    PHI_i << alpha, beta, gamma;

    pf = p_home;
    PHI_f = PHI_home;

    sendJointTraj(pi, pf, PHI_i, PHI_f, startTime, startTime + 4, Ts, ra);

    //First cube detection
    std::cout << "Moving to detection point... " << std::endl;

    pi = p_home;
    PHI_i = PHI_home;

    pf = detectionP;
    PHI_f = detectionPHI;

    // Use joint space trajectory to move the manipulator
    sendJointTraj(pi, pf, PHI_i, PHI_f, startTime + 4, startTime + 8, Ts, ra);

    std::cout << "Aruco detection..." << std::endl;

    geometry_msgs::TransformStamped transformStamped;

    //check to see if transformStamped is initialized, wait 1 seconds for be sure to obatin the correct transformStamped
    //ros::Duration(1).sleep();
    while (ros::ok() && abs(transformStamped.transform.translation.x) < 0.001 && abs(transformStamped.transform.translation.y) < 0.001 && abs(transformStamped.transform.translation.z) < 0.001)
    {
        transformStamped = getArucoTransformStamped(arucoId);
        ros::spinOnce();
        loop_rate.sleep();
    }

    pf << transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z;

    std::cout << "Aruco detected..." << std::endl;

    pf(0) += marginX;
    pf(1) += marginY;
    pf(2) += marginZ;

    sendTrajectory(detectionP, pf, detectionPHI, finalPHI, startTime + 8, startTime + 10, Ts, ra);
}

/**
 * MAIN
 */
int main(int argc, char **argv)
{
    std::cout << "Starting main..." << std::endl;

    // Ros node initialization
    std::cout << "Initializing ROS node..." << std::endl;
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    RobotArm ra(n);

    double Ts = 0.1;
    ros::Rate loop_rate(1 / Ts);

    // Vision system
    cameraSub = n.subscribe("/wrist_rgbd/color/camera_info", 1000, cameraCallback);
    imageSub = n.subscribe("/wrist_rgbd/color/image_raw", 1000, imageCallback);

    joint_state_sub = n.subscribe("/robot/joint_states", 1, jointsCallback);

    createArmClient(ArmClient);

    //Buffer for lookupTransform, computation of aruco relative to robot_base_footprint
    tf2_ros::TransformListener tfListener(tfBuffer);

    while (!joints_done)
    {
        ros::spinOnce();
        loop_rate.sleep();
    };

    //Important 3D positions
    MatrixXd p_vertical_pose(3, 1);
    MatrixXd p_home(3, 1);
    MatrixXd p_blueCube(3, 1);
    MatrixXd p_redCube(3, 1);
    MatrixXd p_greenCube(3, 1);
    MatrixXd p_yellowCube(3, 1);

    p_vertical_pose << 0, 0, 1.2;
    p_blueCube << 0.65, 0.318, 0.613;
    p_greenCube << 0.65, 0.218, 0.613;
    p_redCube << 0.65, -0.473, 0.613;
    p_yellowCube << 0.739, -0.166, 0.727;

    //Important 3D orientations
    MatrixXd PHI_vertical_pose(3, 1);
    MatrixXd PHI_blueCube(3, 1);
    MatrixXd PHI_redCube(3, 1);
    MatrixXd PHI_greenCube(3, 1);
    MatrixXd PHI_yellowCube(3, 1);

    PHI_vertical_pose << 0, 0, 0;
    PHI_blueCube << 0, -PI, -1.311;
    PHI_redCube << 0, -PI, -1.311;
    PHI_greenCube << 0, -PI, -1.311;
    PHI_yellowCube << 0, -PI, -0.7;

    MatrixXd PHI_parallel(3, 1);
    PHI_parallel << 0, -PI, -PI2;

    //Aruco id for each cube
    int blueCubeArucoId = 0;
    int greenCubeArucoId = 2;
    int yellowCubeArucoId = 5;
    int redCubeArucoId = 4;

    // Margin for not crashing with blue and green cube

    double marginX = -0.3;
    double marginY = 0;
    double marginZ = 0.08;

    std::cout << "Moving to vertical configuration... " << std::endl;
    
    control_msgs::FollowJointTrajectoryGoal goal=getVerticalGoal();
    ArmClient->sendGoalAndWait(goal);//TODO spostare in getVerticalGoal
    //TODO, sistemare posizione yellow e blue. Provare a togliere o cambiare la home    
//    pickAndPlaceSingleObject(blueCubeArucoId, p_blueCube, PHI_blueCube, marginX, marginY, marginZ, PHI_parallel, loop_rate, ra, 0.1, true, 0);

    pickAndPlaceSingleObject(greenCubeArucoId, p_greenCube, PHI_greenCube, marginX, marginY, marginZ, PHI_parallel, loop_rate, ra, 0.1, false, 0);

    // Margin for not crashing with yellow cube
    marginX = -0.2;
    marginY = 0;
    marginZ = 0.4;

    pickAndPlaceSingleObject(yellowCubeArucoId, p_yellowCube, PHI_yellowCube, marginX, marginY, marginZ, PHI_yellowCube, loop_rate, ra, 0.1, false, 0);

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}