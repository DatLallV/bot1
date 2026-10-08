/**********************ROS****************************************/
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*********************EIGEN***************************************/
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>

/********************OPENCV LIBRARY********************************/
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"

using namespace std;
using namespace cv;

// 相机内参矩阵（需用标定结果替换）
cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 
    833.5051210437123, 0, 330.5683362465313,
    0, 833.8074347617661, 255.7232077600928, 
    0, 0, 1);

// 相机畸变系数（需用标定结果替换）
cv::Mat dist_coeffs = (cv::Mat_<double>(1,5) << 
    0.04509283803368063, 0.2234215792538128,
    0.004863167176240346, 0.004636669832202732, 0);

cv::Ptr<cv::aruco::Dictionary> dictionary;
double Zc = 1.0;

void callbackImage(const sensor_msgs::ImageConstPtr& msg);
void getMarker(cv::Mat& marker_image);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "axif_tf");
    ros::NodeHandle n;
    
    image_transport::ImageTransport it_(n);
    image_transport::Subscriber image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, callbackImage);
    
    cout << "========================================" << endl;
    cout << "Aruco Marker 检测节点已启动" << endl;
    cout << "订阅话题: /usb_cam/image_raw" << endl;
    cout << "========================================" << endl;
    
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void callbackImage(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    getMarker(cv_ptr->image);
    imshow("Aruco Detection", cv_ptr->image);
    waitKey(1);
}

void getMarker(cv::Mat& marker_image)
{
    vector<int> ids;
    vector< vector<cv::Point2f> > corners;
    vector<cv::Vec3d> rvecs, tvecs;
    
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
    
    if(marker_image.empty()) return;
    
    cv::aruco::detectMarkers(marker_image, dictionary, corners, ids);
    
    if(ids.empty()) return;
    
    cv::aruco::drawDetectedMarkers(marker_image, corners, ids);
    cv::aruco::estimatePoseSingleMarkers(corners, 0.10, camera_matrix, dist_coeffs, rvecs, tvecs);
    
    if(rvecs.empty() || tvecs.empty()) return;
    
    cv::aruco::drawAxis(marker_image, camera_matrix, dist_coeffs, rvecs, tvecs, 0.1);
    
    Zc = tvecs[0][2];
    cout << "========================================" << endl;
    cout << "检测到 Aruco Marker, ID: " << ids[0] << endl;
    cout << "旋转向量 rvec: [" << rvecs[0][0] << ", " << rvecs[0][1] << ", " << rvecs[0][2] << "]" << endl;
    cout << "平移向量 tvec: [" << tvecs[0][0] << ", " << tvecs[0][1] << ", " << tvecs[0][2] << "]" << endl;
    cout << "深度 Zc: " << Zc << " m" << endl;
    cout << "========================================" << endl;
}
