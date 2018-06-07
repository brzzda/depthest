//
// Created by Peter Zborovský on 7.1.2018.
//

#include <Remapping/PoseRemaper.h>

PoseRemaper::PoseRemaper() {
    pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("svo/pose",1, &PoseRemaper::remapCallback, this);
//    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ardrone/std_pose",1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("std_pose",1);
}

PoseRemaper::~PoseRemaper() {

}

//remaps from PoseWithCovarianceStamped to PoseStamped
void PoseRemaper::remapCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg) {
    geometry_msgs::PoseStamped poseToSend;
    poseToSend.header = pose_msg->header;
    poseToSend.pose = pose_msg->pose.pose;
    pose_pub.publish(poseToSend);
}
