//
// Created by peetaa on 27.4.2018.
//

#ifndef DEPTHEST_WINDOWPOSECHANGEVARIANCE_H
#define DEPTHEST_WINDOWPOSECHANGEVARIANCE_H

#include "PositionFilter.h"

class SWPoseChangeFilter : public PositionFilter{
private:
    std::vector<float> distances;
    cv::Mat lastPose;
    float distanceSum;
    float distanceVar;
    int count;
    int windowSize;
    float var;
    float calculateVariance();
    bool first;
public:
    SWPoseChangeFilter();
    ~SWPoseChangeFilter() = default;
    void addPose(cv::Mat pose) override;
    float getVariance() override;

};


#endif //DEPTHEST_WINDOWPOSECHANGEVARIANCE_H
