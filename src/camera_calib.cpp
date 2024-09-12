#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>

using namespace std;

// 定义棋盘的尺寸
int CHECKERBOARD[2]{6,9}; 

std::deque<sensor_msgs::ImagePtr> camera_buffer;
mutex mtx_buffer;
int image_count = 0;

std::vector<std::vector<cv::Point3f>> objpoints;
std::vector<std::vector<cv::Point2f>> imgpoints;
std::vector<cv::Point3f> objp;

void imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
    // 保存图像为文件
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::string filename = "/home/goslam/github/SensorsCalibration/camera_intrinsic/intrinsic_calib/data1/" + std::to_string(image_count) + ".png";
    cv::Mat image_flip = cv_ptr->image;
    cv::flip(image_flip, image_flip, -1);
    cv::imwrite(filename, cv_ptr->image);
    image_count++;
}

// 播放 rosbag
int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh;


    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 100000, imageCallback);

    ros::spin();

    return 0;
}

// 直接接入相机
// int main(int argc, char** argv)
// {
//     // 创建向量以存储每个棋盘图像的 3D 点向量
//     std::vector<std::vector<cv::Point3f>> objpoints;
//     // 创建向量以存储每个棋盘图像的 2D 点向量
//     std::vector<std::vector<cv::Point2f>> imgpoints;

//     // 定义 3D 点的世界坐标
//     std::vector<cv::Point3f> objp;
//     for(int i{0}; i<CHECKERBOARD[1]; i++)
//     {
//         for(int j{0}; j<CHECKERBOARD[0]; j++)
//             objp.push_back(cv::Point3f(j, i, 0));
//     }

//     // 打开相机（0通常是默认相机）
//     cv::VideoCapture cap(0);
//     if (!cap.isOpened()) {
//         std::cerr << "Error: Could not open camera." << std::endl;
//         return -1;
//     }

//     cv::Mat frame, gray;
//     std::vector<cv::Point2f> corner_pts;
//     bool success;
//     int imageCount = 0;
//     const int numImages = 40; // 捕捉的图像数量

//     while (imageCount < numImages) {
//         cap >> frame;
//         if (frame.empty()) {
//             std::cerr << "Error: Failed to capture image." << std::endl;
//             return -1;
//         }

//         cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

//         success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, 
//             cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

//         if (success) {
//             cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
//             cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
//             cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

//             objpoints.push_back(objp);
//             imgpoints.push_back(corner_pts);
//             imageCount++;

//             cv::imshow("Image", frame);
//             cv::waitKey(500); // 等待500毫秒，确保用户可以看到捕捉的图像
//         }
//         else {
//             cv::imshow("Image", frame);
//             cv::waitKey(100); // 等待100毫秒以便进行下一次捕捉
//         }
//     }

//     cv::destroyAllWindows();
//     cap.release(); // 释放相机

//     cv::Mat cameraMatrix, distCoeffs, R, T;
//     cv::Size imageSize(gray.cols, gray.rows);

//     // 通过传递已知 3D 点 (objpoints) 的值 和检测到的角点（imgpoints）对应的像素坐标 实现相机标定
//     cv::calibrateCamera(objpoints, imgpoints, imageSize, cameraMatrix, distCoeffs, R, T);

//     std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
//     std::cout << "distCoeffs : " << distCoeffs << std::endl;
//     std::cout << "Rotation vector : " << R << std::endl;
//     std::cout << "Translation vector : " << T << std::endl;

//     return 0;
// }

