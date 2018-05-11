//
// Created by peetaa on 27.4.2018.
//

#ifndef DEPTHEST_POSEFILTER_H
#define DEPTHEST_POSEFILTER_H


//#include <opencv2/opencv.hpp>
//#include "opencv-3.3.1/opencv2/core/mat.hpp"
#include "opencv-3.3.1/opencv2/core/mat.hpp"

class PositionFilter {
public:
    virtual void addPose(cv::Mat pose) = 0;
    virtual float getVariance() = 0;
};

#endif //DEPTHEST_POSEFILTER_H
