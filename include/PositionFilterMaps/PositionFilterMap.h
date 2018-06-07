//
// Created by peetaa on 27.4.2018.
//

#ifndef DEPTHEST_POSEVARMAP_H
#define DEPTHEST_POSEVARMAP_H

#include <map>
#include "PoseFilterMap.h"
#include "PositionFilterMaps/PoseFilters/PoseFilter.h"

/**
 * wrapper for map of Pose filter
 * the Pose filters of tracked points are accessed through this wrapper.
 */
class PositionFilterMap : public PoseFilterMap {
private:
    std::map<int, PoseFilter> pointVars;

public:
    void addPoint(int id, cv::Mat point);
    float getVariance(int id);
    void eraseMap(int id);
};


#endif //DEPTHEST_POSEVARMAP_H
