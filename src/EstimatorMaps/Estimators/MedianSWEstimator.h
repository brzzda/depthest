//
// Created by peetaa on 22.4.2018.
//

#ifndef DEPTHEST_MEDIANWINDOWPOINTFILTER_H
#define DEPTHEST_MEDIANWINDOWPOINTFILTER_H



#include <vector>
#include <opencv-3.3.1/opencv2/core/mat.hpp>
#include <queue>
#include "PositionEstimator.h"

class MedianSWEstimator : public PositionEstimator {
private:
    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> Z;
    ulong countMed;
    ulong countAvg;
    cv::Mat pose;
    cv::Mat estimatedPose;
    double poseChange;
    bool first;
    int slidingWindowSize;
    cv::Mat calculateEstimatedPose();
public:
    MedianSWEstimator();
    ~MedianSWEstimator();
    void update(cv::Mat pt);
    cv::Mat getEstimate();
    float getPoseChange();
};


#endif //DEPTHEST_MEDIANWINDOWPOINTFILTER_H
