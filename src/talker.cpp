#include <iostream>
#include <sstream>
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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#include "robot_arm2.hpp"
#include "kdl_kinematics.hpp"
#include "cartesian_trajectory.hpp"
#include "aruco_info.hpp"

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
    std::vector<std::vector<cv::Point2f>> corners, rejCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejCandidates);

    // No aruco detected
    if (ids.size() <= 0)
        return;

    // At least one marker has been detected
    ROS_INFO("Cubes detected!");
    // cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

    // Aruco information container
    arucoVec = new std::vector<ArucoInfo>;
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, 0.05, K, D, rvecs, tvecs);
    // draw axis for each marker
    for (int i = 0; i < ids.size(); i++)
    {
        // Save aruco id, rotation and traslation
        arucoVec->push_back(ArucoInfo(ids[i], rvecs[i], tvecs[i]));
        // cv::aruco::drawAxis(imageCopy, K, D, rvecs[i], tvecs[i], 0.1);
    }

    // Show results
    //cv::imshow("out", imageCopy);

    //char key = (char) cv::waitKey(1);
}

void sendTrajectory(MatrixXd pi, MatrixXd pf, MatrixXd PHI_i, MatrixXd PHI_f, double ti, double tf, double Ts, RobotArm ra, ros::Publisher chatter_pub)
{
    std::cout << "Initializing trajectory..." << std::endl;
    CartesianTrajectory *trajectory = new CartesianTrajectory(pi, pf, PHI_i, PHI_f, ti, tf, Ts);
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
        for (int i = 0; i < 6; i++)
        {
            if (target_joints.data[i] > 3.14 || target_joints.data[i] < -3.14)
            {
                check = false;
                break;
            }
        }

        if (check)
        {
            //if (i == 0 || i == round(length/2) || i == length -1) {
            std::cout << "Sending data to Ros" << std::endl;

            double p = 0.0001;
            if (i == 0 || i == length - 1)
            {
                p = 0.0;
            }

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
    ros::Publisher chatter_pub = n.advertise<trajectory_msgs::JointTrajectory>("/robot/arm/pos_traj_controller/command", 10000);

    while (!joints_done)
    {
        ros::spinOnce();
    };

    // Trajectory initial, final and sampling time
    std::cout << "Setting time..." << std::endl;
    double ti = 0.0;
    double tf = 2.0;
    double Ts = 0.1;

    ros::Rate loop_rate(1 / Ts);

    std::cout << "Setting points matrices..." << std::endl;
    MatrixXd pf(3, 1);
    MatrixXd PHI_f(3, 1);

    MatrixXd pi(3, 1);
    MatrixXd PHI_i(3, 1);
    KDL::Frame fr = ra.FKinematics(joints);
    pi << fr.p.x(), fr.p.y(), fr.p.z();
    //@todo verificare il frame di riferimento corretto, l'orientamento iniziale non corrisponde.
    //Il problema potrebbe essere che non è giusto usare GetEulerZYZ oppure non stiamo guardando il giusto tf ('robot_arm_tool0' oppure 'robot_wsg50_center')
    //verificare con rosrun tf tf_echo robot_base_footprint robot_wsg50_center

    //@todo se funziona portare tutto il codice nella funzione sendTrajectory per non replicarlo ad ogni pezzo di traiettoria
    double alpha, beta, gamma;
    fr.M.GetRPY(alpha, beta, gamma);
    PHI_i << alpha, beta, gamma;
    pf << 0, 0, 1.2;
    PHI_f << 0, 0, 0;

    sendTrajectory(pi, pf, PHI_i, PHI_f, ti, tf, Ts, ra, chatter_pub);
    ros::spinOnce();
    loop_rate.sleep();
    pi = pf;
    PHI_i = PHI_f;

    pf << 0.613, 0.318, 0.506;
    //roll+pi/2,pitch-pi/2 rispetto a robot_arm_tool0
    //davanti al cubo blu  -0.282, -3.14, -1.311
    PHI_f << -0.282, -3.14, -1.311;
    ti = 2.0;
    tf = 4.0;
    sendTrajectory(pi, pf, PHI_i, PHI_f, ti, tf, Ts, ra, chatter_pub);
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
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    return 0;
}