//
// Created by peetaa on 11.11.2017.
//

#include <Remapping/ImageRemaper.h>
#include <cv_bridge/cv_bridge.h>
#include <Params.h>

ImageRemaper::ImageRemaper() : it(nh) {
//    image_sub = it.subscribe("/camera/image", 1, &ImageRemaper::imageCallback, this);
    image_sub = it.subscribe("/ardrone/image_raw", 1, &ImageRemaper::imageCallback, this);
    image_sub = it.subscribe(Params::getInstance().getImageTopic(), 1, &ImageRemaper::imageCallback, this);
    image_pub = it.advertise(Params::getInstance().getUndistImateTopic(), 1);
    cam_matrix = Camera::getInstance().getCameraMatrix();
    dist_coefs = Camera::getInstance().getDistCoeffs();
//    std::cout << "remaper" << std::endl;
}

ImageRemaper::~ImageRemaper() {

}

//undistort image
void ImageRemaper::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
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
    cv::Mat temp;
    cv::undistort(cv_ptr->image, temp, cam_matrix, dist_coefs);
    cv_ptr->image = temp;
    image_pub.publish(cv_ptr->toImageMsg());
}
