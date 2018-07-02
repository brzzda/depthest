//
// Created by peetaa on 23.4.2018.
//

#include <EstimatorMaps/MedianEstimatorPointMap.h>

void MedianEstimatorPointMap::update(int id, cv::Mat point) {
    points3D[id].update(point);
}

cv::Mat MedianEstimatorPointMap::getEstimate(int id) {
    return points3D[id].getEstimate();
}

float MedianEstimatorPointMap::getPoseChange(int id) {
    return points3D[id].getPoseChange();
}

void MedianEstimatorPointMap::erase(int id) {
    points3D.erase(id);
}
