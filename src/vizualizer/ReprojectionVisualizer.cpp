//
// Created by peetaa on 19.4.2018.
//

#include <vizualizer/ReprojectionVisualizer.h>
#include <vizualizer/DepthVisualizer.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Camera.h>
#include <Params.h>
#include <HelperFunctions.h>

ReprojectionVisualizer::ReprojectionVisualizer() : it(nh) {
//    image_subscriber = new message_filters::Subscriber<sensor_msgs::Image> (nh, "image_undist", 1000);
    image_subscriber = new message_filters::Subscriber<sensor_msgs::Image> (nh, Params::getInstance().getUndistImateTopic(), 1000);
    pose_and_points_subscriber = new message_filters::Subscriber<depthest::PoseAndPointsStamped> (nh, "pose_and_points", 1000);


    synchronizer = new message_filters::Synchronizer<syncPolicy> (syncPolicy(1000),
                                                                  *pose_and_points_subscriber,
                                                                  *image_subscriber);
    synchronizer->registerCallback(boost::bind(&ReprojectionVisualizer::vizCallBack, this, _1, _2));
    img_pub = it.advertise("image_backproj", 100);
    ROS_INFO_STREAM("Reprojection visualizer created");
}

ReprojectionVisualizer::~ReprojectionVisualizer() {

}



void ReprojectionVisualizer::vizCallBack(const depthest::PoseAndPointsStampedConstPtr &poseAndPointsMsg,
                                  const sensor_msgs::ImageConstPtr &imageMsg) {
    if (poseAndPointsMsg->points.size() == 0) {
        ROS_INFO_STREAM("no points to visualize. dumping visualization");
        return;
    }
    std::vector<cv::Point2d> points2d;
    int frameDisplacement = getFrameDisplacement();
//    std::vector<depthest::Point2DAged> pts2d = points2dMsg->points;
    for (int i = 0; i < poseAndPointsMsg->points.size(); i++) {
        cv::Point2d pt;
        pt.x = poseAndPointsMsg->points[i].x;
        pt.y = poseAndPointsMsg->points[i].y;
        points2d.push_back(pt);
    }
//
    cv::Mat cameraMatrix = Camera::getInstance().getCameraMatrix();
    cv::Mat pose;
    fromStdPose2cvMat(poseAndPointsMsg->pose, pose);

//    cv::Mat points3d(3,1,6);
    cv::Mat points3d;
    fromPointsStamped2cvMat(poseAndPointsMsg->points3D, points3d);
    cv::Mat lastRow = cv::Mat::ones(1, points3d.cols, points3d.type());
    points3d.push_back(lastRow);
    cv::Mat pose_ = pose(cv::Rect(0,0,4,3));
    cv::Mat projMatrix = cameraMatrix * pose_;

    cv::Mat backProjectedPts = projMatrix * points3d;


    makeInhomogenous(backProjectedPts, backProjectedPts);

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);

    cv::Scalar correctPointColor (0, 255, 0);
    cv::Scalar backProjPointColor (255, 0, 0);
    std::vector<cv::Point2d> backProjectedPoints;
    fromCvMat2points2d(backProjectedPts, backProjectedPoints);

    std::vector<double> distances;
//    std::cout << " reprojecting " << std::endl;
    for (int i = 0; i < backProjectedPts.cols; i++) {
        cv::circle(cv_ptr->image, points2d[i], 2, correctPointColor, -1, 8);
        cv::circle(cv_ptr->image, backProjectedPoints[i] , 2, backProjPointColor, -1, 8);

//        cv::line(cv_ptr->image, points2d[i],backProjectedPoints[i], correctPointColor);

//        if (dist < 100) {
//            cv::line(cv_ptr->image, points2d[i],backProjectedPoints[i], correctPointColor);
//        }
    }

    //publish image
    img_pub.publish(cv_ptr->toImageMsg());
}
