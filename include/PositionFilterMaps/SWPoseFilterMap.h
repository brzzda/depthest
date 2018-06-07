//
// Created by peetaa on 27.4.2018.
//

#ifndef DEPTHEST_WINDOWPOSEVARMAP_H
#define DEPTHEST_WINDOWPOSEVARMAP_H

#include <map>
#include "PoseFilterMap.h"
#include "PositionFilterMaps/PoseFilters/SWPoseFilter.h"

/**
 * wrapper for map of Sliding window Pose filter
 * the Pose filters of tracked points are accessed through this wrapper.
 */
class SWPoseFilterMap : public PoseFilterMap{
private:
    std::map<int, SWPoseFilter> pointVars;

public:
    void addPoint(int id, cv::Mat point) override;
    float getVariance(int id) override;
    void eraseMap(int id);

};


#endif //DEPTHEST_WINDOWPOSEVARMAP_H
