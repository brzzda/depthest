//
// Created by peetaa on 22.4.2018.
//

#ifndef DEPTHEST_POINTFILTER_H
#define DEPTHEST_POINTFILTER_H


#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
//#include <ros/ros.h>

class PositionEstimator {
public:
    virtual void update(cv::Mat point) = 0;
    virtual cv::Mat getEstimate() = 0;
    virtual float getPoseChange() = 0;
};

#endif //DEPTHEST_POINTFILTER_H
