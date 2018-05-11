//
// Created by peetaa on 27.4.2018.
//

#ifndef DEPTHEST_POSECHANGEVARIANCECALC_H
#define DEPTHEST_POSECHANGEVARIANCECALC_H

#include <opencv-3.3.1/opencv2/core/mat.hpp>

#include "PositionFilter.h"

/**
 * Pose change filter for monitoring the change in position of a point.
 */
class PoseChangeFilter : public PositionFilter{
private:
    ulong count;

    cv::Mat lastPose;
    double distance;
    double distanceVar;
    double distanceLastVar;
    double distanceSum;
    double distAverage;
    double distLastAverage;

    bool first;
public:
    PoseChangeFilter();
    ~PoseChangeFilter() = default;
    void addPose(cv::Mat pose) override;
    float getVariance() override;
};


#endif //DEPTHEST_POSECHANGEVARIANCECALC_H
