#include "aruco_info.hpp"

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

ArucoInfo::ArucoInfo(int id, cv::Vec3d rvec, cv::Vec3d tvec){
    this->id = id;
    this->rvec = rvec;
    this->tvec = tvec;
    //this->addToTf();
    
    std::cout << "detected aruco: " << id << std::endl;
    std::cout << "pos: " << rvec[0] << "," << rvec[1] << "," << rvec[2];
}

int ArucoInfo::getId(){
    return id;
}

cv::Vec3d ArucoInfo::getRvec(){
    return rvec;
}

cv::Vec3d ArucoInfo::getTvec(){
    return tvec;
}

void ArucoInfo::addToTf() {
    //robot_wrist_rgbd_color_frame oppure robot_wrist_rgbd_color_optical_frame
    //@todo capire se è visibile sto frame da qualche parte
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.frame_id = "robot_wrist_rgbd_color_frame";
    transformStamped.child_frame_id = "aruco_" + std::to_string(id);
    //ROS_INFO("created frame: " +  transformStamped.child_frame_id);
    std::cout << "created frame: " << transformStamped.child_frame_id << std::endl;
    transformStamped.transform.translation.x = tvec[0];
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
}