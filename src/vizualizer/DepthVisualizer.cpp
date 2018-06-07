//
// Created by peetaa on 18.12.2017.
//

#include <vizualizer/DepthVisualizer.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Camera.h>
#include <Params.h>
#include <HelperFunctions.h>

DepthVisualizer::DepthVisualizer() : it(nh){
//    image_subscriber = new message_filters::Subscriber<sensor_msgs::Image> (nh, "image_undist", 1000);
    image_subscriber = new message_filters::Subscriber<sensor_msgs::Image> (nh, Params::getInstance().getUndistImateTopic(), 1000);
    pose_and_points_subscriber = new message_filters::Subscriber<depthest::PoseAndPointsStamped> (nh, "pose_and_points", 1000);

    synchronizer = new message_filters::Synchronizer<syncPolicy> (syncPolicy(1000),
                                                                  *pose_and_points_subscriber,
                                                                  *image_subscriber);
    synchronizer->registerCallback(boost::bind(&DepthVisualizer::vizCallBack, this, _1, _2));

    maxDist = 2;
    minDist = 1;
    img_pub = it.advertise("image_depth", 100);
    ROS_INFO_STREAM("Depth visualizer created");


    distsSum = 0;
    minDistsSum = 0;
    frameCount = 0;
    totalPoints = 0;
    evaluationEnabled = Params::getInstance().getEnabledMeasurement();
}

DepthVisualizer::~DepthVisualizer() {

}

void DepthVisualizer::vizCallBack(const depthest::PoseAndPointsStampedConstPtr &poseAndPointsMsg,
                                          const sensor_msgs::ImageConstPtr &imageMsg) {
    if (poseAndPointsMsg->points.size() == 0 || poseAndPointsMsg->points3D.size() == 0) {
        img_pub.publish(imageMsg);
        ROS_INFO_STREAM("points 2d or points3d are empty. dumping visualization");
        return;
    }

    //camera pose
    cv::Mat pose;
    pose.push_back(poseAndPointsMsg->pose.position.x);
    pose.push_back(poseAndPointsMsg->pose.position.y);
    pose.push_back(poseAndPointsMsg->pose.position.z);
    //get image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
    //all distances
    unsigned long size = poseAndPointsMsg->points3D.size();

    double dist;
    double distSum = 0;
    auto min = DBL_MAX;
    auto max = DBL_MIN;

    for (int i = 0; i < size; i++) {
        //3d point
        cv::Mat pt;
        pt.push_back(poseAndPointsMsg->points3D[i].x);
        pt.push_back(poseAndPointsMsg->points3D[i].y);
        pt.push_back(poseAndPointsMsg->points3D[i].z);
        //corresponding image point
        cv::Point2d ptIm;
        ptIm.x = poseAndPointsMsg->points[i].x;
        ptIm.y = poseAndPointsMsg->points[i].y;
        //paint colored depth dot on corresponding image coordinates
        dist = getDistance(pt, pose);
        distSum += dist;

        cv::circle(cv_ptr->image, ptIm, 2, getColor(remapDistance(dist)), -1, 8);

    }
    totalPoints += size;
    frameCount++;
    distsSum += distSum;
    dists.push_back(distSum);
    int id = static_cast<int>(frameCount / 2) + 1;
    if (evaluationEnabled) {
        ROS_INFO_STREAM("average depth / pt " << distsSum / totalPoints);
        ROS_INFO_STREAM("current depth / pt: " << distSum / size);
    }

    img_pub.publish(cv_ptr->toImageMsg());

}

double DepthVisualizer::getDistance(cv::Mat pt3d, cv::Mat pose) {
    pt3d.convertTo(pt3d, pose.type());
    cv::Mat s;
    cv::subtract(pt3d, pose, s);
    cv::Mat ss;
    cv::transpose(s, ss);
    s = ss * s;
    return sqrt(s.at<double>(0,0));
}



double DepthVisualizer::remapDistance(double distance) {
    double ds = 1021.0;
    //max and minDist values are expected to be properly set
    double dv = maxDist - minDist;
    return (distance - minDist) * ds/dv + 255.0;
}

cv::Scalar DepthVisualizer::getColor(double remapedDistance) {
    if (remapedDistance >= 765) {
        return cv::Scalar(255,1020 - remapedDistance, 0);
    } else if (remapedDistance >= 510) {
        return cv::Scalar(remapedDistance - 510, 255, 0);
    } else if (remapedDistance >= 255) {
        return cv::Scalar(0, 255, 510 - remapedDistance);
    }
    return cv::Scalar(0, remapedDistance, 255);
}

