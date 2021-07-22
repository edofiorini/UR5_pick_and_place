#ifndef ARUCO_INFO
#define ARUCO_INFO

#include <opencv2/aruco.hpp>

class ArucoInfo {
private:
    // Fields
    int id;
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    // void addToTf();

public:
    // Constructor
    ArucoInfo(int id, cv::Vec3d rvec, cv::Vec3d tvec);

    // Getter
    int getId();
    cv::Vec3d getRvec();
    cv::Vec3d getTvec();

    // Methods
    double distance();
    void print();
};

#endif
