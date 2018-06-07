//
// Created by peetaa on 22.4.2018.
//

#ifndef DEPTHEST_POINT3DKALMAN_H
#define DEPTHEST_POINT3DKALMAN_H


#include <opencv2/video.hpp>
#include "PositionEstimator.h"

class KalmanFilterEstimator : public PositionEstimator {
private:
    cv::KalmanFilter KF;
//    cv::Mat state = cv::Mat(3, 1, CV_32F); /* (phi, delta_phi) */
//    cv::Mat processNoise = cv::Mat(3, 1, CV_32F);
//    cv::Mat measurement = cv::Mat::zeros(1, 1, CV_32F);
    cv::Mat estimatedPose;
//    ulong count;
    cv::Mat pose;
//    cv::Mat lastPose;
    float poseChange;
    bool first;
public:
    KalmanFilterEstimator();
    ~KalmanFilterEstimator();
    void init (cv::Mat);
    void update(cv::Mat point);
    cv::Mat getEstimate();
    float getPoseChange();
};


#endif //DEPTHEST_POINT3DKALMAN_H
