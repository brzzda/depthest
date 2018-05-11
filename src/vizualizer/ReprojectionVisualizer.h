//
// Created by peetaa on 19.4.2018.
//

#ifndef DEPTHEST_REPROJECTIONVISUALIZER_H
#define DEPTHEST_REPROJECTIONVISUALIZER_H


#include<depthest/FlowArrayStampedAged.h>
#include<depthest/PointsStamped.h>
#include<depthest/PoseAndPointsStamped.h>
#include<sensor_msgs/Image.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <image_transport/publisher.h>
#include <image_transport/image_transport.h>

/**
 * visualization of reprojected points.
 */
class ReprojectionVisualizer {
private:
    typedef message_filters::sync_policies::ApproximateTime<depthest::PoseAndPointsStamped,
                                                            sensor_msgs::Image> syncPolicy;
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> *image_subscriber;
    message_filters::Subscriber<depthest::PoseAndPointsStamped> *pose_and_points_subscriber;

    message_filters::Synchronizer<syncPolicy> *synchronizer;

    image_transport::Publisher img_pub;
    image_transport::ImageTransport it;
public:
    ReprojectionVisualizer();
    ~ReprojectionVisualizer();
    void vizCallBack(const depthest::PoseAndPointsStampedConstPtr &poseAndPointsMsg,
                     const sensor_msgs::ImageConstPtr &imageMsg);
};


#endif //DEPTHEST_REPROJECTIONVISUALIZER_H
