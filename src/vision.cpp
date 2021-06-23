#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

//#include <rvc/vision.h>
//#include <rvc/region.h>

// Message data
cv_bridge::CvImagePtr imagePtr;
cv::Mat K(3, 3, CV_64F);
cv::Mat D(1, 5, CV_64F);

// Image counter
int imgCount = 0;

// Topic
ros::Publisher dataPub;
ros::Subscriber imageSub, cameraSub;

void cameraCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    // Retrive camera matrix
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            K.at<double>(i, j) = msg->K[i*3+j];
    // This callback is needed to be called just one time
    cameraSub.shutdown();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if(++imgCount < 5)
        return;
    imgCount = 0;

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

    // if at least one marker detected
    if (ids.size() > 0) {
        ROS_INFO("Cubes detected!");
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, K, D, rvecs, tvecs);
        // draw axis for each marker
        for(int i=0; i<ids.size(); i++)
            cv::aruco::drawAxis(imageCopy, K, D, rvecs[i], tvecs[i], 0.1);

    } else // Show rejected candidates
        cv::aruco::drawDetectedMarkers(imageCopy, rejCandidates);
    // Show results
    cv::imshow("out", imageCopy);

    char key = (char) cv::waitKey(1);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "vision");

    ros::NodeHandle n;

    cameraSub = n.subscribe("/wrist_rgbd/color/camera_info", 1000, cameraCallback);
    imageSub = n.subscribe("/wrist_rgbd/color/image_raw", 1000, imageCallback);
    // dataPub = n.advertise<rvc::vision>("rvc_vision", 50000);

    ros::spin();

    return 0;
}
