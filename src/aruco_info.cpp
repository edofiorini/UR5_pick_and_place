#include "aruco_info.hpp"

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

/*#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace Eigen;
*/
/*
aruco 5 frontale
*/

ArucoInfo::ArucoInfo(int id, cv::Vec3d rvec, cv::Vec3d tvec) {
    this->id = id;
    this->rvec = rvec;
    this->tvec = tvec;
    this->addToTf();
    //this->computeTrasform();
}

void ArucoInfo::computeTrasform() {
    
 //p0 = d01+R01*p1
    //d01 tf tra base e camera, traslazione
    //r01 tf tra base e camera, RPY da trasformare in matrice di rotazione
    //p1 = tvec

    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("/robot_base_footprint", "/robot_wrist_rgbd_color_optical_frame", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/robot_base_footprint", "/robot_wrist_rgbd_color_optical_frame", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    //dovrebbe essere la rotation matrix
    //tf::Matrix3x3 rotation = transform.getBasis();

    //dovrebbe essere la traslazione tra i frame
    //tf::Vector3 translation = transform.getOrigin();  

    //transform overloads the "*" operator. You can apply the transform multipling a vector to it
    tf::Vector3 p1 = tf::Vector3(this->tvec[0],this->tvec[1],this->tvec[2]);
    tf::Vector3 p0 = transform*p1;
    this->p << p0.x(), p0.y(), p0.z();
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
        << ", p: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
}

Vector3d ArucoInfo::getP() {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("/robot_base_footprint", "/"+this->arucoFrame, ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/robot_base_footprint",  "/"+this->arucoFrame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return p;
    }
   
    //transform overloads the "*" operator. You can apply the transform multipling a vector to it
    tf::Vector3 p0 = transform.getOrigin();
    this->p << p0.x(), p0.y(), p0.z();
    return p;
}

void ArucoInfo::addToTf() {
    //robot_wrist_rgbd_color_frame oppure robot_wrist_rgbd_color_optical_frame
    //@todo capire se è visibile sto frame da qualche parte
    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "robot_wrist_rgbd_color_optical_frame";
    this->arucoFrame = "aruco_" + std::to_string(id);// + "_" + std::to_string(ros::Time::now().toSec());
    transformStamped.child_frame_id = this->arucoFrame;
    //ROS_INFO("created frame: " +  transformStamped.child_frame_id);
    std::cout << "created frame: " << transformStamped.child_frame_id << std::endl;
    transformStamped.transform.translation.x = -tvec[0];//rotated aruco
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
