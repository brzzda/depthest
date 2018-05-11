//
// Created by peetaa on 21.4.2018.
//

#ifndef DEPTHEST_POINT3DS_H
#define DEPTHEST_POINT3DS_H


#include <vector>
#include <opencv-3.3.1/opencv2/core/mat.hpp>
#include <queue>
#include "PositionEstimator.h"

class MedianEstimator : public PositionEstimator {
private:
    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> Z;
    ulong countMed;
    cv::Mat pose;
    cv::Mat estimatedPose;
    float distance;
    bool first;
    cv::Mat estimatePose();
public:
    MedianEstimator();
    ~MedianEstimator();
    void update(cv::Mat point);
    cv::Mat getEstimate();
    float getPoseChange();
};


#endif //DEPTHEST_POINT3DS_H
