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

// Image counter
int imgCount = 0;

// Topic
ros::Publisher dataPub;
ros::Subscriber imageSub, cameraSub, joint_state_sub;


void cameraCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    // Retrive camera matrix
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            K.at<double>(i, j) = msg->K[i*3+j];
}

void jointsCallback(const sensor_msgs::JointState& msg) {
//   name[]
  //   name[0]: robot_arm_elbow_joint
  //   name[1]: robot_arm_shoulder_lift_joint
  //   name[2]: robot_arm_shoulder_pan_joint
  //   name[3]: robot_arm_wrist_1_joint
  //   name[4]: robot_arm_wrist_2_joint
  //   name[5]: robot_arm_wrist_3_joint
  //   name[6]: robot_back_left_wheel_joint
  //   name[7]: robot_back_right_wheel_joint
  //   name[8]: robot_front_laser_base_joint
  //   name[9]: robot_front_left_wheel_joint
  //   name[10]: robot_front_right_wheel_joint
  //   name[11]: robot_rear_laser_base_joint
  //   name[12]: robot_wsg50_finger_left_joint
  //   name[13]: robot_wsg50_finger_right_joint
// position[]
//   position[0]: 0.351993
//   position[1]: -0.187495
//   position[2]: -1.88308
//   position[3]: 3.1416
//   position[4]: -1.67409
//   position[5]: -0.771451
//   position[6]: 0.00665016
//   position[7]: 8.72015e-05
//   position[8]: 1.06114e-06
//   position[9]: 0.00268441
//   position[10]: 0.00834393
//   position[11]: -5.51283e-08
//   position[12]: -0.00270603
//   position[13]: 0.00270666
    // std::cout << msg.position[0] << std::endl;
    for(int i=0; i<6; i++) {
      
      joints[i] = msg.position[i];
      //std::cout<<"giunto "<<joints[i]<<std::endl;
    }
      joints_done = true;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  // if(++imgCount < 5)
  //     return;
  // imgCount = 0;

  cv::Mat image, imageCopy;
  try {
      // Retrive image from camera topic
      imagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  image = imagePtr->image;
  cv::flip(image, image, 1); 
  image.copyTo(imageCopy);

  // Aruco detection
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f> > corners, rejCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejCandidates);

  if (ids.size() <= 0) {
    // No aruco detected
    return;
  }

  // At least one marker has been detected
  ROS_INFO("Cubes detected!");
  // cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

  // Aruco information container
  arucoVec = new std::vector<ArucoInfo>;
  std::vector<cv::Vec3d> rvecs, tvecs;
  cv::aruco::estimatePoseSingleMarkers(corners, 0.05, K, D, rvecs, tvecs);
  // draw axis for each marker
  for(int i=0; i<ids.size(); i++) {
    // Save aruco id, rotation and traslation
    arucoVec->push_back(ArucoInfo(ids[i], rvecs[i], tvecs[i]));
    // cv::aruco::drawAxis(imageCopy, K, D, rvecs[i], tvecs[i], 0.1);
  }

  // Show results
  //cv::imshow("out", imageCopy);

  //char key = (char) cv::waitKey(1);
}

void sendTrajectory(CartesianTrajectory *trajectory, float Ts, RobotArm ra, ros::Publisher chatter_pub) {
 
  /*
  double giunti_belli[6];
  for(int i = 0; i < 6; i++) {
    giunti_belli[i] = joints[i];
  }
*/
  float length = trajectory->get_length();
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
    "robot_arm_wrist_3_joint"
  };

  for(int i=0; i<length; i++) {
    double vel_[6];
    double acc_[6];
    bool* flag = new bool(false);

    std::cout 
      << "\npoints " 
      << trajectory->dataPosition.coeff(0,i) 
      << " " 
      << trajectory->dataPosition.coeff(1,i)
      << " " 
      << trajectory->dataPosition.coeff(2,i)
      << " " 
      << trajectory->dataPosition.coeff(3,i)
      << " " 
      << trajectory->dataPosition.coeff(4,i)
      << " " 
      << trajectory->dataPosition.coeff(5,i)
      << std::endl;      

    target_joints = ra.IKinematics(
      trajectory->dataPosition.coeff(0,i), 
      trajectory->dataPosition.coeff(1,i), 
      trajectory->dataPosition.coeff(2,i), 
      trajectory->dataPosition.coeff(3,i), // @todo inspect the use of nan in orientation
      trajectory->dataPosition.coeff(4,i), 
      trajectory->dataPosition.coeff(5,i),
      joints,
      trajectory->dataVelocities,
      i,
      trajectory->dataAcceleration,
      length,
      vel_,
      acc_,
      flag
    );

    //std::cout << " flag : " << *flag << std::endl;
    bool check = true;
    for(int i = 0; i < 6; i++) {
      if(target_joints.data[i] > 3.14 || target_joints.data[i] < -3.14) {
        check = false;
        break;
      }
    }

    if(*flag && check){
      //if (i == 0 || i == round(length/2) || i == length -1) {    
      std::cout << "Sending data to Ros" << std::endl;

      float p = 0.0001f;
      if (i == 0 || i == length-1) {
        p = 0.0f;
      }

      trajectory_msgs::JointTrajectoryPoint point;
      point.positions.resize(6);
      point.positions = { 
        target_joints.data[0], 
        target_joints.data[1], 
        target_joints.data[2], 
        target_joints.data[3], 
        target_joints.data[4],
        target_joints.data[5]
      };
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
      point.time_from_start = ros::Duration(i*Ts);
      points.push_back(point);
    }
   
    for(int j = 0; j < 6; j++)
      previous_vel_[j] = vel_[j];
  }

  msg.points = points;
  chatter_pub.publish(msg);
}

/**
 * MAIN
 */
int main(int argc, char **argv) {
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

  while(!joints_done) {
    ros::spinOnce();
  };

  // Trajectory initial, final and sampling time
  std::cout << "Setting time..." << std::endl;
  float ti = 0.0f;
  float tf = 2.0f;
  float Ts = 0.1f;

  ros::Rate loop_rate(1/Ts);

  std::cout << "Setting points matrices..." << std::endl;
  MatrixXd pi(3, 1);
  MatrixXd pf(3, 1);
  //MatrixXd c(3, 1);
  MatrixXd PHI_i(3, 1);
  MatrixXd PHI_f(3, 1);

  //pi << 0.491, -0.008, 1.134;
 
  pf << 0.643, 0.327, 0.506;
  //c << 1, 1, 1;
  //PHI_i << 0, 0, 0;
  PHI_f << -1.572, 0.004, -1.592;  

  KDL::Frame fr = ra.FKinematics(joints);

  pi << fr.p.x(), fr.p.y(), fr.p.z();  
  //@todo verificare il frame di riferimento corretto, l'orientamento iniziale non corrisponde.
  //Il problema potrebbe essere che non è giusto usare GetEulerZYZ oppure non stiamo guardando il giusto tf ('robot_arm_tool0' oppure 'robot_wsg50_center')
  //verificare con rosrun tf tf_echo robot_base_footprint robot_wsg50_center

  //@todo se funziona portare tutto il codice nella funzione sendTrajectory per non replicarlo ad ogni pezzo di traiettoria
  double alpha,beta,gamma;
  fr.M.GetEulerZYZ(alpha,beta,gamma);
  PHI_i << alpha,beta,gamma;

  std::cout << "Initializing trajectory..." << std::endl;
  CartesianTrajectory *initialTrajectory = new CartesianTrajectory(pi, pf, PHI_i, PHI_f, ti, tf, Ts);
  std::cout << "Trajectory initialized!" << std::endl;

  sendTrajectory(initialTrajectory, Ts, ra, chatter_pub);
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

  ros::spinOnce();

  loop_rate.sleep();
 
  return 0;
}