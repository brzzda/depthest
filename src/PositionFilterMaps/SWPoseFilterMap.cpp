//
// Created by peetaa on 27.4.2018.
//

#include <PositionFilterMaps/SWPoseFilterMap.h>

void SWPoseFilterMap::addPoint(int id, cv::Mat point) {
    pointVars[id].addPose(point);
}

float SWPoseFilterMap::getVariance(int id) {
    return pointVars[id].getVariance();
}

void SWPoseFilterMap::eraseMap(int id) {
    pointVars.erase(id);
}
