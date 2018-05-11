//
// Created by peetaa on 23.4.2018.
//

#ifndef DEPTHEST_MEDIANFILTERPOINTMAP_H
#define DEPTHEST_MEDIANFILTERPOINTMAP_H


#include "EstimatorMaps/Estimators/MedianEstimator.h"
#include "EstimatorPointMap.h"

/**
 * wrapper for map of Median Position estimator
 * each point is stored, accessed or erased  here.
 */
class MedianEstimatorPointMap : public EstimatorPointMap {
private:
    std::map<int, MedianEstimator> points3D;
public:
    void update(int id, cv::Mat point) override;
    cv::Mat getEstimate(int id) override;
    float getPoseChange(int id) override;
    void erase(int id) override;
};


#endif //DEPTHEST_MEDIANFILTERPOINTMAP_H
