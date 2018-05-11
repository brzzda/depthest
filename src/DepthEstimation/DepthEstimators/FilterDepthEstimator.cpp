//
// Created by peetaa on 18.11.2017.
//

#include "FilterDepthEstimator.h"
#include "HelperFunctions.h"


FilterDepthEstimator::FilterDepthEstimator() : cameraMatrix(Camera::getInstance().getCameraMatrix()),
                                                   params(Params::getInstance()){
    lastPose = cv::Mat::zeros(4, 4, CV_32FC1);
    frameDisplacement = params.getTriangWindowSize();
    evaluationEnabled = params.getEnabledMeasurement();

    varianceFilterType = params.getFilterName();
    poseVarianceThresh = varianceFilterType == POSE_VARIANCE ? params.getPoseVarianceThresth() : params.getPoseChangeVarianceThresth();
    points3D = pointMapFactory.getPointMap(params.getEstimatorName());
    poseVariacesMap = poseVarianceMapFactory.getPoseVariancesMap(params.getFilterName());
    emptyFrameCount = 0;
    filteredOutPoints = 0;
}

FilterDepthEstimator::~FilterDepthEstimator()
{

}

void FilterDepthEstimator::addPoint(cv::Point2f point, int id) {
    points[id].push(point);
    if (points[id].size() == frameDisplacement) {
        firstViewPoints2D.push_back(points[id].front());
        secondViewPoints2D.push_back(point);
        ids.push_back(id);
        points[id].pop();
    }
}

void FilterDepthEstimator::deletePoint(int id) {
    points.erase(id);
    points3D->erase(id);
//    poseVariances.erase(id);
    poseVariacesMap->eraseMap(id);
}

void FilterDepthEstimator::estimateDepth(cv::Mat &out) {
    if (firstViewPoints2D.empty()) return;
    if (poses.size() >= frameDisplacement) {
        cv::Mat pts1(firstViewPoints2D);
        cv::Mat pts2(secondViewPoints2D);
        cv::Mat proj1 = cameraMatrix * poses.front();
        cv::Mat proj2 = cameraMatrix * poses.back();
        poses.pop();
        //triangulation
        cv::triangulatePoints(proj1, proj2, pts1, pts2, out);

        makeInhomogenous(out, out);
        update3DPoints(out, out);
        if(out.cols == 0) {
            emptyFrameCount++;
            return;
        }

        cv::Mat lastRow = cv::Mat::ones(1, out.cols, out.type());
        out.push_back(lastRow);
        cv::Mat ptss1(lastFirstViewPoints2D);
        cv::Mat ptss2(lastSecondViewPoints2D);


        if (evaluationEnabled)
            evaluate(proj1, proj2, ptss1, ptss2, out);
        ids.clear();
    }
}

void FilterDepthEstimator::addPose(cv::Mat pose) {
    cv::Mat pose_ = pose(cv::Rect(0,0,4,3));
    poses.push(pose_);
}

std::vector<cv::Point2f> FilterDepthEstimator::getSecondViewPoints2D() {
    return lastSecondViewPoints2D;
}

void FilterDepthEstimator::evaluate(cv::Mat proj1, cv::Mat proj2, cv::Mat pts1, cv::Mat pts2, cv::Mat pts3D) {
    evaluator.setReprojValues(proj1, proj2, pts1, pts2, pts3D);
    evaluator.calculateReprojError();
    evaluator.calculateDistanceError();
    ROS_INFO_STREAM("current RE/pt: " << evaluator.getReprojectionError()/pts3D.cols);
    ROS_INFO_STREAM("current ME/pt: " << evaluator.getCurrentFramePoseChangeError()/pts3D.cols);
    ROS_INFO_STREAM("current frame displacement: " << evaluator.getCurrentFrameDisplacement());
    ROS_INFO_STREAM("-----");
    ROS_INFO_STREAM("Total points: " << evaluator.getTotalPointCount());
    ROS_INFO_STREAM("frames Count: " << evaluator.getFrameCount());
    ROS_INFO_STREAM("Spare frames count: " << evaluator.getSparfeFrameCount());
    ROS_INFO_STREAM("Empty frames count: " << emptyFrameCount);
    ROS_INFO_STREAM("points rejected total: " << filteredOutPoints);
    ROS_INFO_STREAM("Average Frame Displacement : " << evaluator.getAverageFrameDisplacement());
    ROS_INFO_STREAM("-----");
    ROS_INFO_STREAM("Total Reprojection Error: " << evaluator.getTotalReprojectionError());
    ROS_INFO_STREAM("Total movement: " << evaluator.getTotalDistance());
    ROS_INFO_STREAM("-----");

}

std::vector<cv::Point2f> FilterDepthEstimator::getFirstViewPoints2D() {
    return lastFirstViewPoints2D;
}

void FilterDepthEstimator::update3DPoints(cv::Mat in, cv::Mat& out) {
    int belowCount = 0;

    lastSecondViewPoints2D.clear();
    lastFirstViewPoints2D.clear();

    if(in.cols < 1) return;
    out = cv::Mat();


    for (int i = 0; i < in.cols; i++) {
        points3D->update(ids[i], in.col(i));
        poseVariacesMap->addPoint(ids[i], points3D->getEstimate(ids[i]));
        if ((poseVariacesMap->getVariance(ids[i]) < poseVarianceThresh)  && ((poseVariacesMap->getVariance(ids[i]) != 0))) {

            lastSecondViewPoints2D.push_back(secondViewPoints2D[i]);
            lastFirstViewPoints2D.push_back(firstViewPoints2D[i]);
            if (out.empty()) {
                out.push_back(points3D->getEstimate(ids[i]));
            } else {
                cv::hconcat(out, points3D->getEstimate(ids[i]), out);
            }
            if (evaluationEnabled) {
                evaluator.addDistance(points3D->getPoseChange(ids[i]));
            }
            belowCount++;
        }

    }
    filteredOutPoints += in.cols - belowCount;

    secondViewPoints2D.clear();
    firstViewPoints2D.clear();

}

void FilterDepthEstimator::enableEvaluation(bool enabled) {
    evaluationEnabled = enabled;
}
