//
// Created by peetaa on 28.10.2017.
//

#include <Params.h>
#include <vizualizer/Visualizer3D.h>
#include <HelperFunctions.h>


Visualizer3D::Visualizer3D()
{
    std::cout << "creating path visualizer node" << std::endl;
    path_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    drone_pose_sub = nh_.subscribe(Params::getInstance().getOdometryTopic(),100, &Visualizer3D::poseCallback, this);
    points_sub = nh_.subscribe("triangulatedPoints", 100, &Visualizer3D::pointsCallback, this);
    rotx = (cv::Mat_<double>(4,4) << 1,0,0,0,  0,0,-1,0,  0,1,0,0,   0,0,0,1);
    roty = (cv::Mat_<double>(4,4) << -1,0,0,0,  0,1,0,0,  0,0,-1,0,  0,0,0,1);
    rotz = (cv::Mat_<double>(4,4) << 0,-1,0,0,  1,0,0,0,  0,0,1,0,  0,0,0,1);
    rot = rotx;
//    rot = rotz  * rotx;
//    rot = roty * rotx;
}

Visualizer3D::~Visualizer3D()
{

}

void Visualizer3D::poseCallback(geometry_msgs::PoseStampedConstPtr msg) {
    visualization_msgs::Marker poseToSend2;
    poseToSend2.header = msg->header;
    poseToSend2.pose.orientation.w = 1.0;
    poseToSend2.header.frame_id = "/map";
    poseToSend2.action = visualization_msgs::Marker::ADD;
    poseToSend2.type = visualization_msgs::Marker::LINE_LIST;
    poseToSend2.scale.x = 0.01;
    poseToSend2.scale.y = 0.01;
    poseToSend2.scale.z = 0.01;
    poseToSend2.color.g = 1.0f;
    poseToSend2.color.a = 1.0;
    poseToSend2.ns = "pose";
    poseToSend2.id = pose_ii++;
    pose_ii = pose_ii > maxPtsInPath ? 0 : pose_ii;
    cv::Mat out(4,4,5);
    fromStdPose2cvMat(msg->pose, out);
    out = rot * out;

    cv::Mat ptt2 = (cv::Mat_<double>(4,1) << 0, -0.1, 0, 1);
    cv::Mat ptt = (cv::Mat_<double>(4,1) << 0, 0, 0, 1);

    ptt2 = out * ptt2;
    ptt = out * ptt;
    makeInhomogenous(ptt2, ptt2);
    makeInhomogenous(ptt, ptt);

    geometry_msgs::Point point;
    geometry_msgs::Point point2;

    fromCvMat2pointsMsg(ptt2, point);
    fromCvMat2pointsMsg(ptt, point2);

    poseToSend2.points.push_back(point2);

    poseToSend2.points.push_back(point);
    path_pub.publish(poseToSend2);

}

void Visualizer3D::pointsCallback(depthest::PointsStampedConstPtr msg) {
    visualization_msgs::Marker poseToSend;
    poseToSend.header = msg->header;

    poseToSend.pose.orientation.w = 1.0;
    poseToSend.header.frame_id = "/map";
    poseToSend.action = visualization_msgs::Marker::ADD;
    poseToSend.type = visualization_msgs::Marker::POINTS;
    poseToSend.scale.x = 0.01;
    poseToSend.scale.y = 0.01;
    poseToSend.scale.z = 0.01;
    poseToSend.color.r = 1.0f;
    poseToSend.color.a = 1.0;
    poseToSend.lifetime = ros::Duration();
    poseToSend.ns = "points";
    poseToSend.id = maxPtsInPath + 1;

    poseToSend.points = msg->points;
    path_pub.publish(poseToSend);
}
