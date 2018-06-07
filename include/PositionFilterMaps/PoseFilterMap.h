//
// Created by peetaa on 27.4.2018.
//

#ifndef DEPTHEST_POSEVARIANCEMAP_H
#define DEPTHEST_POSEVARIANCEMAP_H

#include "opencv2/core/mat.hpp"

/**
 * interface for Position filters wrappers.
 */
class PoseFilterMap {
public:
    virtual void addPoint(int id, cv::Mat point) = 0;
    virtual float getVariance(int id) = 0;
    virtual void eraseMap(int id) = 0;
};

#endif //DEPTHEST_POSEVARIANCEMAP_H
