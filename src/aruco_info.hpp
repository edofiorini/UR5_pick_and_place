#include <opencv2/aruco.hpp>

class ArucoInfo {
private:
    // Fields
    int id;
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    void addToTf();

public:
    // Constructor
    ArucoInfo(int id, cv::Vec3d rvec, cv::Vec3d tvec);

    // Getter
    int get_id();
    cv::Vec3d get_rvec();
    cv::Vec3d get_tvec();
};