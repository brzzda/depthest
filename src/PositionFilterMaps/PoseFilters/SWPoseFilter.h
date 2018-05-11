//
// Created by peetaa on 27.4.2018.
//

#ifndef DEPTHEST_WINDOWPOSEVARIANCE_H
#define DEPTHEST_WINDOWPOSEVARIANCE_H

#include "PositionFilter.h"

class SWPoseFilter :public PositionFilter {
private:
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    float xSum;
    float ySum;
    float zSum;
    float xVar;
    float yVar;
    float zVar;
    int count;
    int windowSize;
    float var;
    float calculateVariance();
public:
    SWPoseFilter();
    ~SWPoseFilter() = default;
    void addPose(cv::Mat pose) override;
    float getVariance() override;
};


#endif //DEPTHEST_WINDOWPOSEVARIANCE_H
