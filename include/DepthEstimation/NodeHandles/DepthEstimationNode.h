//
// Created by Peter Zborovský on 29.10.2017.
//

#ifndef DEPTHEST_POSEANDOBJECTSUBSCRIBER_H
#define DEPTHEST_POSEANDOBJECTSUBSCRIBER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <depthest/ObjectsStamped.h>
#include <depthest/FlowArrayStamped.h>
#include <depthest/Point2D.h>
#include <depthest/PointsStamped.h>
#include <depthest/PoseAndPointsStamped.h>
#include <depthest/FlowArrayStampedAged.h>
#include <DepthEstimation/DepthEstimators/DepthEstimator.h>
#include <DepthEstimation/DepthEstimatorFactory.h>

/**
 * Depthestimation node
 */
class DepthEstimationNode {
private:
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
            depthest::ObjectsStamped> ObjAndPoseSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
            depthest::FlowArrayStamped> FlowAndPoseSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
            depthest::FlowArrayStampedAged> AgedFlowSyncPolicy;

    DepthEstimatorFactory deFactory;
    DepthEstimator* de;
    ros::NodeHandle nh;

    ros::Publisher pointsPublisher;
    ros::Publisher pointsAndPosePublisher;

    message_filters::Subscriber<geometry_msgs::PoseStamped> *pose_filter;
    message_filters::Subscriber<depthest::ObjectsStamped> *obj_filter;
    message_filters::Subscriber<depthest::FlowArrayStamped> *flow_filter;
    message_filters::Subscriber<depthest::FlowArrayStampedAged> *aged_flow_filter;

    message_filters::Synchronizer<ObjAndPoseSyncPolicy> *object_sync;
    message_filters::Synchronizer<FlowAndPoseSyncPolicy> *flow_sync;
    message_filters::Synchronizer<AgedFlowSyncPolicy> *aged_flow_sync;

public:
    DepthEstimationNode();
    ~DepthEstimationNode(){delete pose_filter; delete obj_filter; delete flow_filter;
                  delete object_sync; delete flow_sync; delete aged_flow_filter;
                  delete aged_flow_sync;}
    void poseAndPointsCallback(const geometry_msgs::PoseStampedConstPtr &poseMsg,
                               const depthest::FlowArrayStampedAgedConstPtr &flowMsg);

};


#endif //DEPTHEST_POSEANDOBJECTSUBSCRIBER_H
