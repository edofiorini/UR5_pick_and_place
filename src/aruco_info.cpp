#include "aruco_info.hpp"

ArucoInfo::ArucoInfo(int id, cv::Vec3d rvec, cv::Vec3d tvec){
    this->id = id;
    this->rvec = rvec;
    this->tvec = tvec;
}

int ArucoInfo::get_id(){
    return id;
}

cv::Vec3d ArucoInfo::get_rvec(){
    return rvec;
}

cv::Vec3d ArucoInfo::get_tvec(){
    return tvec;
}
