//
// Created by peetaa on 27.4.2018.
//

#include "PositionFilterMap.h"

void PositionFilterMap::addPoint(int id, cv::Mat point) {
    pointVars[id].addPose(point);
}

float PositionFilterMap::getVariance(int id) {
    return pointVars[id].getVariance();
}

void PositionFilterMap::eraseMap(int id) {
    pointVars.erase(id);
}
