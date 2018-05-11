//
// Created by peetaa on 27.4.2018.
//

#include "NoFilteringDepthEstimator.h"
#include "HelperFunctions.h"

NoFilteringDepthEstimator::NoFilteringDepthEstimator() : cameraMatrix(Camera::getInstance().getCameraMatrix()),
                                                   params(Params::getInstance()){
    type_ = 0;
    lastPose = cv::Mat::zeros(4, 4, CV_32FC1);
    lastPoints = cv::Mat::zeros(2, 1, CV_8UC1);
    lastProjMatrix = cv::Mat::zeros(3, 4, CV_32FC1);
    frameDisplacement = params.getTriangWindowSize();
    evaluationEnabled = params.getEnabledMeasurement();
//    std::string filterParam = "/depthest/point_filter";
//    if(!ros::param::get(filterParam, POINT_FILTER_TYPE)) {
//        ROS_INFO_STREAM("could not find parameter point_filter, setting to default: value " << MEDIAN_POINT_FILTER);
//        POINT_FILTER_TYPE = MEDIAN_POINT_FILTER;
//    }
//    nh.param<int>(filterParam, POINT_FILTER_TYPE, MEDIAN_POINT_FILTER); //median = 2;
    points3D = pointMapFactory.getPointMap(params.getEstimatorName());
//    points3D = pointMapFactory.getPointMap(MEDIAN_WINDOW_PONT_FILTER);
//    ROS_INFO_STREAM("Found params: " << filterParam << ": " << POINT_FILTER_TYPE);
}

NoFilteringDepthEstimator::~NoFilteringDepthEstimator()
{

}

void NoFilteringDepthEstimator::addPoint(cv::Point2f point, int id) {
    points[id].push(point);
    if (points[id].size() == frameDisplacement) {
        firstViewPoints2D.push_back(points[id].front());
        secondViewPoints2D.push_back(point);
        ids.push_back(id);
        points[id].pop();
    }
}

void NoFilteringDepthEstimator::deletePoint(int id) {
    points.erase(id);
    points3D->erase(id);
}

void NoFilteringDepthEstimator::estimateDepth(cv::Mat &out) {
    if (firstViewPoints2D.empty()) return;
    if (poses.size() >= frameDisplacement) {
        cv::Mat pts1(firstViewPoints2D);
        cv::Mat pts2(secondViewPoints2D);
        cv::Mat proj1 = cameraMatrix * poses.front();
        cv::Mat proj2 = cameraMatrix * poses.back();
        firstViewPoints2DOut.clear();
        secondViewPoints2DOut.clear();
        firstViewPoints2DOut.swap(firstViewPoints2D);
        secondViewPoints2DOut.swap(secondViewPoints2D);
        secondViewPoints2D.clear();
        firstViewPoints2D.clear();
        poses.pop();
        //triangulation
        cv::triangulatePoints(proj1, proj2, pts1, pts2, out);

        makeInhomogenous(out, out);
        update3DPoints(out, out);

        cv::Mat lastRow = cv::Mat::ones(1, out.cols, out.type());
        out.push_back(lastRow);

        if (evaluationEnabled)
            evaluate(proj1, proj2, pts1, pts2, out);
        ids.clear();
    }
//    else { /* do absolutely nothing and watch if someone cares */}
}

void NoFilteringDepthEstimator::addPose(cv::Mat pose) {
    cv::Mat pose_ = pose(cv::Rect(0,0,4,3));
    poses.push(pose_);
}

std::vector<cv::Point2f> NoFilteringDepthEstimator::getSecondViewPoints2D() {
    return secondViewPoints2DOut;
}

void NoFilteringDepthEstimator::evaluate(cv::Mat proj1, cv::Mat proj2, cv::Mat pts1, cv::Mat pts2, cv::Mat pts3D) {
    evaluator.setReprojValues(proj1, proj2, pts1, pts2, pts3D);
    evaluator.calculateReprojError();
    evaluator.calculateDistanceError();
//    evaluator.setPoseChangeValues(points3D, ids);
//    evaluator.calculateReprojError2();
//    ROS_INFO_STREAM("-----");
//    ROS_INFO_STREAM("Reprojection error: " << evaluator.getReprojectionError());
//    ROS_INFO_STREAM("Average rep. error: " << evaluator.getAverageReprojectionError());
//    ROS_INFO_STREAM("Median rep. error: " << evaluator.getReprojectionMedianError());
//    ROS_INFO_STREAM("-----");
//    ROS_INFO_STREAM("current pose change error: " << evaluator.getCurrentFramePoseChangeError());
//    ROS_INFO_STREAM("Average pose change error: " << evaluator.getAverageDistance());
//    ROS_INFO_STREAM("Median pose change error: " << evaluator.getMedianDistance());
//    ROS_INFO_STREAM("Median error2: " << evaluator.getMedianError2());
    ROS_INFO_STREAM("current RE/pt: " << evaluator.getReprojectionError()/pts3D.cols);
    ROS_INFO_STREAM("current ME/pt: " << evaluator.getCurrentFramePoseChangeError()/pts3D.cols);
    ROS_INFO_STREAM("current frame displacement: " << evaluator.getCurrentFrameDisplacement());
//    ROS_INFO_STREAM("Average pose change error: " << evaluator.getAverageDistance());
//    ROS_INFO_STREAM("Median pose change error: " << evaluator.getMedianDistance());
//    ROS_INFO_STREAM("Median error2: " << evaluator.getMedianError2());

    ROS_INFO_STREAM("-----");
    ROS_INFO_STREAM("Total points: " << evaluator.getTotalPointCount());
    ROS_INFO_STREAM("frames Count: " << evaluator.getFrameCount());
//    ROS_INFO_STREAM("Average points per frame: " << evaluator.getAveragePointCount()); // allpointCount / frame count
    ROS_INFO_STREAM("Spare frames count: " << evaluator.getSparfeFrameCount());
//    ROS_INFO_STREAM("average rejected points per frame: " << filteredOutPoints / evaluator.getFrameCount());
    ROS_INFO_STREAM("Average Frame Displacement : " << evaluator.getAverageFrameDisplacement());
    ROS_INFO_STREAM("-----");
    ROS_INFO_STREAM("Total Reprojection Error: " << evaluator.getTotalReprojectionError());
    ROS_INFO_STREAM("Total movement: " << evaluator.getTotalDistance());
//    ROS_INFO_STREAM("Total Average Reprojection Error: " << evaluator.getTotalAverageReprojectionError()); //totalRE / allPointCount
//    ROS_INFO_STREAM("Total average movement: " << evaluator.getTotalAverageDistance()); // total dist / all point count
    ROS_INFO_STREAM("-----");
//    ROS_INFO_STREAM("Total average ratio: " << evaluator.getTotalAverageDistance() / evaluator.getTotalAverageReprojectionError());
//    ROS_INFO_STREAM("Total ratio: " << evaluator.getTotalDistance() / evaluator.getTotalReprojectionError());
    ROS_INFO_STREAM("-----");
    ROS_INFO_STREAM("Total median of av rep Er " << evaluator.getMedianOfAverageReprojectionErrors());
    ROS_INFO_STREAM("Total median of av dist " << evaluator.getMedianOfAverageDistances());
    ROS_INFO_STREAM("-----");

}

std::vector<cv::Point2f> NoFilteringDepthEstimator::getFirstViewPoints2D() {
    return firstViewPoints2DOut;
}

void NoFilteringDepthEstimator::update3DPoints(cv::Mat in, cv::Mat& out) {
    if(in.cols < 1) return;
    out = cv::Mat();
    points3D->update(ids[0], in.col(0));
    out.push_back(points3D->getEstimate(ids[0]));
    for(int i = 1; i < in.cols; i++) {
        points3D->update(ids[i], in.col(i));
        if(evaluationEnabled){
            evaluator.addDistance(points3D->getPoseChange(ids[i]));
        }
        cv::hconcat(out, points3D->getEstimate(ids[i]), out);
    }
}

void NoFilteringDepthEstimator::enableEvaluation(bool enabled) {
    evaluationEnabled = enabled;
}