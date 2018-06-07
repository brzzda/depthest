//
// Created by peetaa on 27.4.2018.
//

#ifndef DEPTHEST_POSEVARIANCEMAPFACTORY_H
#define DEPTHEST_POSEVARIANCEMAPFACTORY_H

#include "PositionFilterMaps/PoseChangeFilterMap.h"
#include "PositionFilterMaps/PositionFilterMap.h"
#include "PositionFilterMaps/SWPoseChangeFilterMap.h"
#include "PositionFilterMaps/SWPoseFilterMap.h"
#include "PositionFilterMaps/PoseFilterMap.h"

class PositionFilterMapFactory {
private:
    PoseChangeFilterMap pcvm;
    PositionFilterMap pvm;
    SWPoseFilterMap wpvm;
    SWPoseChangeFilterMap wpcvm;
    std::string pointFilterType;
public:
    PositionFilterMapFactory();
    PoseFilterMap* getPoseVariancesMap(std::string varianceFilterType);

};


#endif //DEPTHEST_POSEVARIANCEMAPFACTORY_H
