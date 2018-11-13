//
// Created by peetaa on 22.4.2018.
//

#ifndef DEPTHEST_POINTFILTER_H
#define DEPTHEST_POINTFILTER_H


#include <opencv2/opencv.hpp>
<<<<<<< HEAD:include/EstimatorMaps/Estimators/PositionEstimator.h
#include <opencv2/core/mat.hpp>
=======
#include "opencv2/core/mat.hpp"
>>>>>>> 40db694c8c80faf5f02e43472b8efa6d6c640ed1:src/EstimatorMaps/Estimators/PositionEstimator.h
//#include <ros/ros.h>

class PositionEstimator {
public:
    virtual void update(cv::Mat point) = 0;
    virtual cv::Mat getEstimate() = 0;
    virtual float getPoseChange() = 0;
};

#endif //DEPTHEST_POINTFILTER_H
