//
// Created by peetaa on 27.4.2018.
//

#include "SWPoseChangeFilterMap.h"

void SWPoseChangeFilterMap::addPoint(int id, cv::Mat point) {
    pointVars[id].addPose(point);
}

float SWPoseChangeFilterMap::getVariance(int id) {
    return pointVars[id].getVariance();
}

void SWPoseChangeFilterMap::eraseMap(int id) {
    pointVars.erase(id);
}
