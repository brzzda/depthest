//
// Created by peetaa on 22.4.2018.
//

#ifndef DEPTHEST_DUMMYPOINTFILTER_H
#define DEPTHEST_DUMMYPOINTFILTER_H


#include <opencv2/core/mat.hpp>
#include "PositionEstimator.h"

class DummyEstimator : public PositionEstimator{
private:
    cv::Mat pose;
    bool first;
    float distance;
public:
    DummyEstimator();
    ~DummyEstimator();
    void update(cv::Mat point) override;
    cv::Mat getEstimate() override;
    float getPoseChange() override;
};


#endif //DEPTHEST_DUMMYPOINTFILTER_H
