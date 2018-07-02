//
// Created by peetaa on 11.11.2017.
//

#ifndef DEPTHEST_IMAGEREMAPER_H
#define DEPTHEST_IMAGEREMAPER_H


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/mat.hpp>
#include "Camera.h"

/**
 * serves to remap the raw image to undistorted image
 * listen to topic stored in ROS param 'image',
 * uses camera matrices to undistort the image and consequently
 * publish undistorted image on topic stored in ROS param 'undist_image'
 */
class ImageRemaper {
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
//    Camera& camera = Camera::getInstance();
    cv::Mat cam_matrix;
    cv::Mat dist_coefs;

public:
    ImageRemaper();
    ~ImageRemaper();
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};


#endif //DEPTHEST_IMAGEREMAPER_H
