//
// Created by peetaa on 23.4.2018.
//

#include <EstimatorMaps/DummyEstimatorPointMap.h>

void DummyEstimatorPointMap::update(int id, cv::Mat point) {
    points3D[id].update(point);
}

cv::Mat DummyEstimatorPointMap::getEstimate(int id) {
    return points3D[id].getEstimate();
}

float DummyEstimatorPointMap::getPoseChange(int id) {
    return points3D[id].getPoseChange();
}

void DummyEstimatorPointMap::erase(int id) {
    points3D.erase(id);
}
