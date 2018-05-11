//
// Created by peetaa on 22.4.2018.
//

#include <HelperFunctions.h>
#include "EstimatorPointMapFactory.h"

EstimatorPointMap* EstimatorPointMapFactory::getPointMap(std::string pointMapType) {
    if (pointMapType == KALMAN_POINT_FILTER) {
        return &kpf;
    } else if (pointMapType == MEDIAN_POINT_FILTER){
        return &mpf;
    } else if (pointMapType == MEDIAN_WINDOW_PONT_FILTER) {
        return &mwpf;
    }
    return &dpf;
}
