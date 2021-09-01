#ifndef ARUCO_INFO
#define ARUCO_INFO

#include <opencv2/aruco.hpp>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <tf/transform_listener.h>

using namespace Eigen;

class ArucoInfo {
private:
    // Fields
    int id;
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    Vector3d p;
    MatrixXd phi;
    std::string arucoFrame;

    void addToTf();

public:
    // Constructor
    ArucoInfo(int id, cv::Vec3d rvec, cv::Vec3d tvec);

    // Getter
    int getId();
    cv::Vec3d getRvec();
    cv::Vec3d getTvec();
    Vector3d getP();
    MatrixXd getPhi();
    
    // Methods
    double distance();
    void print();
    //void addToTf();
    //void computeTrasform();
};

#endif
