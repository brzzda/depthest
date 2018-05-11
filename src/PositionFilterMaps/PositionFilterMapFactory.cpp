//
// Created by peetaa on 27.4.2018.
//

#include <Params.h>
#include "PositionFilterMapFactory.h"
#include "HelperFunctions.h"

PositionFilterMapFactory::PositionFilterMapFactory() {
    pointFilterType = Params::getInstance().getEstimatorName();
}

PoseFilterMap *PositionFilterMapFactory::getPoseVariancesMap(std::string varianceFilterType) {
    if (varianceFilterType == POSE_VARIANCE && pointFilterType == MEDIAN_WINDOW_PONT_FILTER) {
        ROS_INFO_STREAM("PVMF window pose variance filter");
        return &wpvm;
    }
    if (varianceFilterType == POSE_VARIANCE) {
        ROS_INFO_STREAM("PVMF pose variance filter");
        return &pvm;
    }
    if (pointFilterType == MEDIAN_WINDOW_PONT_FILTER) {
        ROS_INFO_STREAM("PVMF window pose variance change filter");
        return &wpcvm;
    }
    ROS_INFO_STREAM("PVMF pose variance change filter");
    return &pcvm;
}
