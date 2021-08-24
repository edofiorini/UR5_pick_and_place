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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#include "robot_arm2.hpp"
#include "kdl_kinematics.hpp"
#include "cartesian_trajectory.hpp"
#include "joint_pol_traj.hpp"
#include "aruco_info.hpp"

// PI costants
#define PI M_PI     // pi
#define PI2 M_PI_2  // pi/2

using namespace Eigen;

bool joints_done = false;

// Message data
cv_bridge::CvImagePtr imagePtr;
cv::Mat K(3, 3, CV_64F);
cv::Mat D(1, 5, CV_64F);
std::vector<ArucoInfo> *arucoVec;

double joints[6];

// Topic
ros::Publisher dataPub;
ros::Subscriber imageSub, cameraSub, joint_state_sub;


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
    std::vector< std::vector<cv::Point2f> > corners, rejCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejCandidates);

    arucoVec = new std::vector<ArucoInfo>;
    // No aruco detected
    if (ids.size() <= 0)
        return;

    // At least one marker has been detected
    ROS_INFO("Cubes detected!");
    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

    // Aruco information container
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, 0.03, K, D, rvecs, tvecs);
    // draw axis for each marker
    for (int i = 0; i < ids.size(); i++)
    {
        // Save aruco id, rotation and traslation
        arucoVec->push_back(ArucoInfo(ids[i], rvecs[i], tvecs[i]));
        cv::aruco::drawAxis(imageCopy, K, D, rvecs[i], tvecs[i], 0.1);
    }

    // Show results
    // cv::imshow("out", imageCopy);
    
    // char key = (char) cv::waitKey(1);
}

ArucoInfo* closestCube(int id) {
    double minDist = 1000;
    ArucoInfo* closest = NULL;
    
    for (int i = 0; i<arucoVec->size(); i++) {
        double dist = arucoVec->at(i).distance();
        if (arucoVec->at(i).getId() == id && dist < minDist) {
            minDist = dist;
            closest = &(arucoVec->at(i));
        }
    }

    return closest;
}

void sendTrajectory(MatrixXd pi, MatrixXd pf, MatrixXd PHI_i, MatrixXd PHI_f, double ti, double tf, double Ts, RobotArm ra, ros::Publisher chatter_pub)
{
    std::cout << "Initializing trajectory..." << std::endl;
    CartesianTrajectory *trajectory;
    trajectory = new CartesianTrajectory(pi, pf, PHI_i, PHI_f, ti, tf, Ts);
    std::cout << "Trajectory initialized!" << std::endl;

    int length = trajectory->get_length();
    double previous_vel_[6];
    KDL::JntArray target_joints;
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;

    trajectory_msgs::JointTrajectory msg;
    msg.joint_names = {
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

    msg.points = points;
    chatter_pub.publish(msg);
}

void sendJointTraj(MatrixXd pi, MatrixXd pf, MatrixXd PHI_i, MatrixXd PHI_f, double ti, double tf, double Ts, RobotArm ra, ros::Publisher chatter_pub){

    JointPolTraj *trajectory;
    trajectory = new JointPolTraj(pi, pf, PHI_i, PHI_f, ra, joints, 6, ti, tf, Ts);
    MatrixXd jointPos = trajectory->getJointPos();
    MatrixXd jointVel = trajectory->getJointVel();

    trajectory_msgs::JointTrajectory msg;
    msg.joint_names = {
        "robot_arm_shoulder_pan_joint",
        "robot_arm_shoulder_lift_joint",
        "robot_arm_elbow_joint",
        "robot_arm_wrist_1_joint",
        "robot_arm_wrist_2_joint",
        "robot_arm_wrist_3_joint"};

    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    for (int i = 0; i<trajectory->getSamples(); i++) {
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
        point.velocities = {
            jointVel.coeff(0, i),
            jointVel.coeff(1, i),
            jointVel.coeff(2, i),
            jointVel.coeff(3, i),
            jointVel.coeff(4, i),
            jointVel.coeff(5, i)};

        points.push_back(point);
    }

    msg.points = points;
    chatter_pub.publish(msg);
}

bool isAtFinalPosition(RobotArm ra, MatrixXd pf, MatrixXd PHI_f){
    MatrixXd pOfNow(3, 1);
    MatrixXd PHI_ofNow(3, 1);
    double alpha, beta, gamma;

    MatrixXd error(1,1);
    error << 0.001;


    KDL::Frame frameOfNow = ra.FKinematics(joints);
    pOfNow << frameOfNow.p.x(), frameOfNow.p.y(), frameOfNow.p.z();
    frameOfNow.M.GetRPY(alpha, beta, gamma);

    PHI_ofNow << alpha, beta, gamma;

    //std::cout << pOfNow << "  -  " << pf << std::endl;

    if(abs((pOfNow-pf).coeff(0,0))<error.coeff(0,0) &&
        abs((pOfNow-pf).coeff(1,0))<error.coeff(0,0) &&
        abs((pOfNow-pf).coeff(2,0))<error.coeff(0,0))
        //abs((PHI_ofNow-PHI_f).coeff(0,0))<error.coeff(0,0) &&
        //abs((PHI_ofNow-PHI_f).coeff(1,0))<error.coeff(0,0) &&
        //abs((PHI_ofNow-PHI_f).coeff(2,0))<error.coeff(0,0))
        return true;
    else
        return false;
}

void goInitPos(ros::Publisher chatter_pub) {

    std::vector<trajectory_msgs::JointTrajectoryPoint> points;

    trajectory_msgs::JointTrajectory msg;
    msg.joint_names = {
        "robot_arm_shoulder_pan_joint",
        "robot_arm_shoulder_lift_joint",
        "robot_arm_elbow_joint",
        "robot_arm_wrist_1_joint",
        "robot_arm_wrist_2_joint",
        "robot_arm_wrist_3_joint"
    };

    double pos[] {0, -0.8, -PI2};

    trajectory_msgs::JointTrajectoryPoint point;
    for (int i=0;i<3;i++) {
        point.positions.resize(6);
        point.positions = {0,pos[i],0,0,0,0};
        point.time_from_start = ros::Duration(i*2);
        points.push_back(point);
    }    

    msg.points = points;
    chatter_pub.publish(msg);
}


bool isAtHome(){
    if ((joints[1] + PI2) < 0.001)
        return true;
    else
        return false;
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

    // Vision system
    cameraSub = n.subscribe("/wrist_rgbd/color/camera_info", 1000, cameraCallback);
    imageSub = n.subscribe("/wrist_rgbd/color/image_raw", 1000, imageCallback);
    // dataPub = n.advertise<rvc::vision>("rvc_vision", 50000);

    joint_state_sub = n.subscribe("/robot/joint_states", 1, jointsCallback);
    
    ros::Publisher chatter_pub = n.advertise<trajectory_msgs::JointTrajectory>("/robot/arm/pos_traj_controller/command", 1000);

    while (!joints_done)
    {
        ros::spinOnce();
    };

    //Important 3D positions
    MatrixXd p_home(3, 1);
    MatrixXd p_blueCube(3, 1);
    MatrixXd p_redCube(3, 1);
    MatrixXd p_greenCube(3, 1);
    MatrixXd p_yellowCube(3, 1);

    p_home << 0, 0, 1.2;
    p_blueCube << 0.55, 0.318, 0.613; //0.506
    p_redCube << 0.513, -0.473, 0.572; // collision problems
    p_greenCube << 0,0,0;
    p_yellowCube << 0.949, -0.176, 0.974;
    //0.692, 0.721, 0.032, 0.005
    
    //Important 3D orientations
    MatrixXd PHI_home(3, 1);
    MatrixXd PHI_blueCube(3, 1);
    MatrixXd PHI_redCube(3, 1);
    MatrixXd PHI_greenCube(3, 1);
    MatrixXd PHI_yellowCube(3, 1);

    PHI_home << 0, 0, 0;
    PHI_blueCube << 0, -PI, -1.311; //-1.852, -1.567, -1.311 real value
    PHI_redCube << 0, -PI, -1.311;
    PHI_greenCube << 0, -PI, -1.311;
    PHI_yellowCube << -3.103, -0.053, 3.179; //-3.103, -0.053, 0.039 real value 
    
    double Ts = 0.1;
    ros::Rate loop_rate(1 / Ts);

    // Initial Position
    std::cout << "Moving to initial configuration... " << std::endl;
    goInitPos(chatter_pub);
    while (ros::ok() && !isAtHome()) {
        std::cout << "joint1 " << joints[1] << " " << joints[1] + PI2 << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }

    // First cube detection
    std::cout << "Moving to first cube... " << std::endl;

    MatrixXd pf(3, 1);
    MatrixXd PHI_f(3, 1);
    MatrixXd pi(3, 1);
    MatrixXd PHI_i(3, 1);

    KDL::Frame fr = ra.FKinematics(joints);
    pi << fr.p.x(), fr.p.y(), fr.p.z();
   
    //rosrun tf tf_echo robot_base_footprint robot_arm_tool0
    double alpha, beta, gamma;
    fr.M.GetRPY(alpha, beta, gamma);
    PHI_i << alpha, beta, gamma;

    ros::spinOnce();
    loop_rate.sleep();

    double ti = 0.0;
    double tf = 2.0;
  
    pf = p_blueCube;
    PHI_f = PHI_blueCube;

    // Use joint space trajectory to move the manipulator
    sendJointTraj(pi, pf, PHI_i, PHI_f, ti, tf, Ts, ra, chatter_pub);
    while(ros::ok() && !isAtFinalPosition(ra,pf,PHI_f)) {
        ros::spinOnce();
    };

    // Find the nearest front aruco (id = 5)
    std::cout << "Aruco detection..." << std::endl;
    int arucoId = 5;
    ArucoInfo* arucoCube = NULL;

    while(ros::ok() && !arucoCube){
        ros::spinOnce();
        loop_rate.sleep();
        arucoCube = closestCube(arucoId);
    }
    pf = arucoCube->getP();
    arucoCube->print();

    //pf(0) -= 0.1;
    //pf(1,0) = pf(1,0)+0.2;
    //pf(2) += 0.03;

    //sendTrajectory(p_blueCube, pf, PHI_blueCube, PHI_f, ti, tf, Ts, ra, chatter_pub);
    //sendJointTraj(p_blueCube, pf, PHI_blueCube, PHI_f, ti, tf, Ts, ra, chatter_pub);
      
    while(ros::ok()) {
        ros::spinOnce();
    }
    
    while(ros::ok() && !isAtFinalPosition(ra,pf,PHI_f)){
        ros::spinOnce();
    }

    return 0;

    fr = ra.FKinematics(joints);
    pi << fr.p.x(), fr.p.y(), fr.p.z();
   
    //rosrun tf tf_echo robot_base_footprint robot_arm_tool0
    alpha, beta, gamma;
    fr.M.GetRPY(alpha, beta, gamma);
    PHI_i << alpha, beta, gamma;

    ros::spinOnce();
    loop_rate.sleep();

    pf = p_blueCube;
    PHI_f = PHI_blueCube;
    sendTrajectory(pi, pf, PHI_i, PHI_f, ti, tf, Ts, ra, chatter_pub);
    while(ros::ok() && !isAtFinalPosition(ra,pf,PHI_f)){
        ros::spinOnce();
    };


    

    
    // std::cout << dataAcceleration << std::endl;

    /*
  ros::AsyncSpinner spinner(2);

  InitialPose arm;

  spinner.start();
  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
*/
    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}