//
// Created by peetaa on 23.4.2018.
//

#include "MedianSWEstimatorPointMap.h"

void MedianSWEstimatorPointMap::update(int id, cv::Mat point) {
    points3D[id].update(point);
}

cv::Mat MedianSWEstimatorPointMap::getEstimate(int id) {
    return points3D[id].getEstimate();
}

float MedianSWEstimatorPointMap::getPoseChange(int id) {
    return points3D[id].getPoseChange();
}

void MedianSWEstimatorPointMap::erase(int id) {
    points3D.erase(id);
}
