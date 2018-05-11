//
// Created by Peter Zborovský on 7.1.2018.
//

#ifndef DEPTHEST_POSEREMAPPER_H
#define DEPTHEST_POSEREMAPPER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
/**
 * remaps posewith covariance to std_pose
 * convenience class for remapping SVO pose message
 */
class PoseRemaper {
private:
    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    ros::Publisher pose_pub;
public:
    PoseRemaper();
    ~PoseRemaper();
    void remapCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);
};


#endif //DEPTHEST_POSEREMAPPER_H
