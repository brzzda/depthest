//
// Created by peetaa on 27.4.2018.
//

#include "PoseChangeFilterMap.h"

void PoseChangeFilterMap::addPoint(int id, cv::Mat point) {
    pointVars[id].addPose(point);
}

float PoseChangeFilterMap::getVariance(int id) {
    return pointVars[id].getVariance();
}

void PoseChangeFilterMap::eraseMap(int id) {
    pointVars.erase(id);
}
