//
// Created by peetaa on 23.4.2018.
//

#ifndef DEPTHEST_POINTFILTERMAP_H
#define DEPTHEST_POINTFILTERMAP_H

#include <opencv2/opencv.hpp>
#include "opencv-3.3.1/opencv2/core/mat.hpp"

/**
 * interface for wrapping Position estimators
 */
class EstimatorPointMap {
public:
    virtual void update(int id, cv::Mat point) = 0;
    virtual cv::Mat getEstimate(int id) = 0;
    virtual float getPoseChange(int id) = 0;
    virtual void erase(int id) = 0;
};

#endif //DEPTHEST_POINTFILTERMAP_H
