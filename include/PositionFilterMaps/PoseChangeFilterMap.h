//
// Created by peetaa on 27.4.2018.
//

#ifndef DEPTHEST_POSECHANGEVARMAP_H
#define DEPTHEST_POSECHANGEVARMAP_H


#include <PositionFilterMaps/PoseFilters/PoseChangeFilter.h>
#include <map>
#include "PoseFilterMap.h"

/**
 * wrapper for map of Pose change filter
 * the Pose change filters of tracked points are accessed through this wrapper.
 */
class PoseChangeFilterMap : public PoseFilterMap {
private:
    std::map<int, PoseChangeFilter> pointVars;

public:
    void addPoint(int id, cv::Mat point);
    float getVariance(int id);
    void eraseMap(int id);

};


#endif //DEPTHEST_POSECHANGEVARMAP_H
