#include "aruco_info.hpp"

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

/*
aruco 5 frontale
*/

ArucoInfo::ArucoInfo(int id, cv::Vec3d rvec, cv::Vec3d tvec){
    this->id = id;
    this->rvec = rvec;
    this->tvec = tvec;

    this->computeTrasform();
    //this->addToTf();
}

void ArucoInfo::computeTrasform(){
    tf::TransformListener tfListener;
    tf::StampedTransform transform;

    try{
        tfListener.lookupTransform(
            "/robot_base_footprint",
            "/robot_wrist_rgbd_color_optical_frame",
            ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    // transform.getBasis() * tvec + transform.getOrigin();

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

double ArucoInfo::distance() {
    return cv::norm(tvec);
}

void ArucoInfo::print() {
    std::cout << "ID: " << id
        << ", R: " << rvec[0] << ", " << rvec[1] << ", " << rvec[2]
        << ", t: " << tvec[0] << ", " << tvec[1] << ", " << tvec[2] << std::endl;
}

/*
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
*/
