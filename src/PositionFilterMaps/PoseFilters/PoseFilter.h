//
// Created by peetaa on 27.4.2018.
//

#ifndef DEPTHEST_POSEVARIANCECALC_H
#define DEPTHEST_POSEVARIANCECALC_H

#include "PositionFilter.h"
/**
 * Pose filter for monitoring the change in feature coordinates
 */
class PoseFilter : public PositionFilter {

private:
    ulong count;
    float poseVar;
    float x;
    float y;
    float z;
    float xVar;
    float yVar;
    float zVar;
    float xLastVar;
    float yLastVar;
    float zLastVar;
    float xAverage;
    float yAverage;
    float zAverage;
    float xLastAverage;
    float yLastAverage;
    float zLastAverage;

    cv::Mat lastPose;

public:
    PoseFilter();
    ~PoseFilter() = default;
    void addPose(cv::Mat pose) override;
    float getVariance() override;
};


#endif //DEPTHEST_POSEVARIANCECALC_H
