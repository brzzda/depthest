//
// Created by peetaa on 23.4.2018.
//

#include <EstimatorMaps/KalmanFilterEstimatorPointMap.h>

void KalmanFilterEstimatorPointMap::update(int id, cv::Mat point) {
    points3D[id].update(point);
}

cv::Mat KalmanFilterEstimatorPointMap::getEstimate(int id) {
    return points3D[id].getEstimate();
}

float KalmanFilterEstimatorPointMap::getPoseChange(int id) {
    return points3D[id].getPoseChange();
}

void KalmanFilterEstimatorPointMap::erase(int id) {
    points3D.erase(id);
}
