//
// Created by Peter Zborovský on 29.10.2017.
//

#include <DepthEstimation/NodeHandles/DepthEstimationNode.h>
#include <tf2_msgs/TFMessage.h>
#include <eigen3/Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.h>
#include <opencv2/core/eigen.hpp>
#include <HelperFunctions.h>


DepthEstimationNode::DepthEstimationNode() {


    pose_filter = new message_filters::Subscriber<geometry_msgs::PoseStamped> (nh, Params::getInstance().getOdometryTopic(), 1);
    aged_flow_filter = new message_filters::Subscriber<depthest::FlowArrayStampedAged> (nh, Params::getInstance().getFeaturesTopic(), 1);

    aged_flow_sync = new message_filters::Synchronizer<AgedFlowSyncPolicy> (AgedFlowSyncPolicy(1000), *pose_filter, *aged_flow_filter);
    aged_flow_sync->registerCallback(boost::bind(&DepthEstimationNode::poseAndPointsCallback, this, _1, _2));

    pointsPublisher = nh.advertise<depthest::PointsStamped>("/triangulatedPoints", 1);
    pointsAndPosePublisher = nh.advertise<depthest::PoseAndPointsStamped>("/pose_and_points", 1);

    de = deFactory.getDepthEstimator(Params::getInstance().getFilterName());

    ros::spin();

}


void DepthEstimationNode::poseAndPointsCallback(const geometry_msgs::PoseStampedConstPtr &poseMsg,
                                                const depthest::FlowArrayStampedAgedConstPtr &flowMsg) {
//    ROS_INFO_STREAM("------------AGED FLOW CALLBACK----------------");


    if(flowMsg->points.size() == 0)
        return;

    //----------------------points----------------------//
    std::vector<depthest::Point2DAged> pts = flowMsg->points;
    for(int i = 0; i < pts.size() ; i++)
    {
        if(pts[i].age == -1) {
            de->deletePoint(pts[i].id);
        } else {
            cv::Point2f point;
            point.x = pts[i].x;
            point.y = pts[i].y;
            de->addPoint(point, pts[i].id);
        }
    }

    int type = 0;
    cv::Mat res;

    //-----------------------pose-----------------//
    cv::Mat pose(4, 4, CV_32FC1);
    fromStdPose2cvMat(poseMsg->pose,pose);
    de->addPose(pose);
    depthest::PoseAndPointsStamped ppmsg;
    ppmsg.pose = poseMsg->pose; //??? donno
    //--------------estimate depth----------------//
    de->estimateDepth(res);
    fromCvPoints2f2Points2D(de->getSecondViewPoints2D(), ppmsg.points);

    //set message and publish it

    depthest::PointsStamped msgOut;
    msgOut.header.frame_id = flowMsg->header.frame_id;
    msgOut.header.seq = flowMsg->header.seq;
    msgOut.header.stamp.nsec = flowMsg->header.stamp.nsec;
    msgOut.header.stamp.sec = flowMsg->header.stamp.sec;
    fromCvMat2pointsMsg(res, msgOut.points);
    pointsPublisher.publish(msgOut);

    ppmsg.header.frame_id = poseMsg->header.frame_id;
    ppmsg.header.seq = poseMsg->header.seq;
    ppmsg.header.stamp.nsec = poseMsg->header.stamp.nsec;
    ppmsg.header.stamp.sec = poseMsg->header.stamp.sec;
    fromCvMat2pointsMsg(res, ppmsg.points3D);
    pointsAndPosePublisher.publish(ppmsg);
}

