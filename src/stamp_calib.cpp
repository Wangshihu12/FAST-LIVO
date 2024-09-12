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

    // stamp: 
    // secs: 1725471664
    // nsecs:    574000
    // frame_id: "head_camera"


    std::string filename = "/home/goslam/Documents/stamp_calib/image/" + std::to_string(img_msg->header.stamp.sec) + "." + std::to_string(img_msg->header.stamp.nsec) + ".png";
    cv::Mat image_flip = cv_ptr->image;
    // cv::flip(image_flip, image_flip, -1);
    cv::imwrite(filename, image_flip);
    // image_count++;
}

// 播放 rosbag
int main(int argc, char** argv)
{
    ros::init(argc, argv, "stamp_calib_node");
    ros::NodeHandle nh;


    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 100000, imageCallback);

    ros::spin();

    return 0;
}