//
// Created by peetaa on 27.4.2018.
//

#ifndef DEPTHEST_WINDOWPOSECHANGEVARMAP_H
#define DEPTHEST_WINDOWPOSECHANGEVARMAP_H

#include <map>
#include "PoseFilterMap.h"
#include "PositionFilterMaps/PoseFilters/SWPoseChangeFilter.h"

/**
 * wrapper for map of Sliding Window Pose change filter
 * the Pose change filters of tracked points are accessed through this wrapper.
 */
class SWPoseChangeFilterMap : public PoseFilterMap{
private:
    std::map<int, SWPoseChangeFilter> pointVars;

public:
    void addPoint(int id, cv::Mat point);
    float getVariance(int id);
    void eraseMap(int id);

};


#endif //DEPTHEST_WINDOWPOSECHANGEVARMAP_H
