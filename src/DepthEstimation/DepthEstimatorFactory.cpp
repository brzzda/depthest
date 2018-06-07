//
// Created by Peter Zborovský on 8.11.2017.
//

#include <DepthEstimation/DepthEstimatorFactory.h>
#include <HelperFunctions.h>

DepthEstimator* DepthEstimatorFactory::getDepthEstimator(std::string type)
{
    if (type == NONE) {
        return &sde;
    }
    return &fde;
}
