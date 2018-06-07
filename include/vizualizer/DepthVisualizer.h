//
// Created by peetaa on 14.12.2017.
//

#ifndef DEPTHEST_DEPTHVISUALIZER_H
#define DEPTHEST_DEPTHVISUALIZER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <image_transport/publisher.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/mat.hpp>
#include <depthest/PoseAndPointsStamped.h>
#include <sensor_msgs/Image.h>
#include <depthest/FlowArrayStampedAged.h>
#include <depthest/PointsStamped.h>

/**
 * Visualization fo Depth informatin
 */
class DepthVisualizer {
private:
    typedef message_filters::sync_policies::ApproximateTime<depthest::PoseAndPointsStamped,
            sensor_msgs::Image> syncPolicy;

    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> *image_subscriber;
    message_filters::Subscriber<depthest::PoseAndPointsStamped> *pose_and_points_subscriber;

    message_filters::Synchronizer<syncPolicy> *synchronizer;

    image_transport::Publisher img_pub;
    image_transport::ImageTransport it;

    double minDist;
    double maxDist;
    double minDistsSum;
    double distsSum;

    std::vector<double> dists;
    std::vector<double> minDists;
    double frameCount;
    ulong totalPoints;
    bool evaluationEnabled;

    double getDistance(cv::Mat pt3d, cv::Mat pose);
    double remapDistance(double distance);
    cv::Scalar getColor(double remapedDistance);

public:
    DepthVisualizer();
    ~DepthVisualizer();
    void vizCallBack(const depthest::PoseAndPointsStampedConstPtr &poseAndPointsMsg,
                     const sensor_msgs::ImageConstPtr &imageMsg);

};

#endif //DEPTHEST_DEPTHVISUALIZER_H
