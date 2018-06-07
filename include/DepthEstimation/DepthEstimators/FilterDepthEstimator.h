//
// Created by peetaa on 18.11.2017.
//

#ifndef DEPTHEST_DISPLACEMENTESTIMATOR_H
#define DEPTHEST_DISPLACEMENTESTIMATOR_H

//#include "ros/ros.h"
#include <EstimatorMaps/EstimatorPointMapFactory.h>
#include <Params.h>
#include <PositionFilterMaps/PoseFilterMap.h>
#include <PositionFilterMaps/PositionFilterMapFactory.h>
#include "DepthEstimation/DepthEstimators/DepthEstimator.h"
#include "Evaluation/ErrorCalc.h"
#include "EstimatorMaps/Estimators/MedianEstimator.h"
#include "EstimatorMaps/Estimators/MedianSWEstimator.h"
#include "EstimatorMaps/Estimators/KalmanFilterEstimator.h"

#include <iostream>
#include <fstream>
#include <string>

/**
 * Depth estimator that use Position filters and Position estimators
 */
class FilterDepthEstimator : public DepthEstimator {
private:
    cv::Mat lastPose;
    cv::Mat cameraMatrix;

    Params& params;

    ros::NodeHandle nh;

    std::queue<cv::Mat> poses;
    std::map<int, std::queue<cv::Point2f>> points;
    PositionFilterMapFactory poseVarianceMapFactory;
    PoseFilterMap* poseVariacesMap;
    EstimatorPointMapFactory pointMapFactory;
    EstimatorPointMap* points3D;

    std::vector<cv::Point2f> secondViewPoints2D;
    std::vector<cv::Point2f> firstViewPoints2D;
    std::vector<cv::Point2f> lastSecondViewPoints2D;
    std::vector<cv::Point2f> lastFirstViewPoints2D;
    std::vector<int> ids;


    int emptyFrameCount;
    int frameDisplacement;
    int filteredOutPoints;
    double poseVarianceThresh;
    bool evaluationEnabled;
    ErrorCalc evaluator;
    std::string varianceFilterType;
public:
    FilterDepthEstimator();
    ~FilterDepthEstimator();
    void addPoint(cv::Point2f point, int id) override;
    void deletePoint(int id) override;
    void estimateDepth(cv::Mat &out) override;
    void addPose(cv::Mat pose) override;
    void evaluate(cv::Mat proj1, cv::Mat proj2, cv::Mat pts1, cv::Mat pts2, cv::Mat pts3D);
    std::vector<cv::Point2f> getFirstViewPoints2D() override;
    std::vector<cv::Point2f> getSecondViewPoints2D() override;
    void update3DPoints(cv::Mat in, cv::Mat &out);
    void enableEvaluation(bool enabled) override;
};


#endif //DEPTHEST_DISPLACEMENTESTIMATOR_H
