//
// Created by peetaa on 22.4.2018.
//

#ifndef DEPTHEST_POINTFILTERFACTORY_H
#define DEPTHEST_POINTFILTERFACTORY_H


#include "EstimatorMaps/EstimatorPointMap.h"
#include "EstimatorMaps/DummyEstimatorPointMap.h"
#include "EstimatorMaps/KalmanFilterEstimatorPointMap.h"
#include "EstimatorMaps/MedianEstimatorPointMap.h"
#include "EstimatorMaps/MedianSWEstimatorPointMap.h"

class EstimatorPointMapFactory {
private:
    MedianSWEstimatorPointMap mwpf;
    MedianEstimatorPointMap mpf;
    KalmanFilterEstimatorPointMap kpf;
    DummyEstimatorPointMap dpf;

public:
    EstimatorPointMapFactory(){};
    EstimatorPointMap* getPointMap(std::string pointMapType);
};


#endif //DEPTHEST_POINTFILTERFACTORY_H
