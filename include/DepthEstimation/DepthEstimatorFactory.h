//
// Created by peetaa on 8.11.2017.
//

#ifndef DEPTHEST_DEPTHESTIMATORFACTORY_H
#define DEPTHEST_DEPTHESTIMATORFACTORY_H

#include <DepthEstimation/DepthEstimators/DepthEstimator.h>
#include <DepthEstimation/DepthEstimators/NoFilteringDepthEstimator.h>
#include <DepthEstimation/DepthEstimators/FilterDepthEstimator.h>

/**
 * factory for creation of all DepthEstimator-s
 */
class DepthEstimatorFactory {
private:
    NoFilteringDepthEstimator sde;
    FilterDepthEstimator fde;
public:
    DepthEstimatorFactory() = default;;
    DepthEstimator* getDepthEstimator(std::string type);
};


#endif //DEPTHEST_DEPTHESTIMATORFACTORY_H
