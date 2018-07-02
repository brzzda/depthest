//
// Created by peetaa on 28.10.2017.
//

#ifndef DEPTHEST_PATHVISUALIZER_H
#define DEPTHEST_PATHVISUALIZER_H


#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv2/core/mat.hpp>
#include <depthest/PointsStamped.h>


class Visualizer3D {
private:
    visualization_msgs::Marker points, line_strip, line_list;

    ros::NodeHandle nh_;
    ros::Publisher path_pub;
    ros::Publisher pose_pub;
    ros::Subscriber drone_pose_sub;
    ros::Subscriber point_sub;
    ros::Subscriber points_sub;

    uint pose_i;
    uint pose_ii;
    float f = 0.0;
    int maxPtsInPath = 200;
    cv::Mat rotx;
    cv::Mat roty;
    cv::Mat rotz;
    cv::Mat rot;

public:
    Visualizer3D();
    ~Visualizer3D();
    void poseCallback(geometry_msgs::PoseStampedConstPtr msg);
    void pointsCallback(depthest::PointsStampedConstPtr msg);

};


#endif //DEPTHEST_PATHVISUALIZER_H
