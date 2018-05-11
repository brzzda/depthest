//
// Created by peetaa on 23.4.2018.
//

#ifndef DEPTHEST_MEDIANWINDOWFILTERPOINTMAP_H
#define DEPTHEST_MEDIANWINDOWFILTERPOINTMAP_H


#include "EstimatorMaps/Estimators/MedianSWEstimator.h"
#include "EstimatorPointMap.h"

/**
 * wrapper for map of Median Sliding Window Position estimator
 * each point is stored, accessed or erased  here.
 */
class MedianSWEstimatorPointMap : public EstimatorPointMap {
private:
    std::map<int, MedianSWEstimator> points3D;
public:
    void update(int id, cv::Mat point) override;
    cv::Mat getEstimate(int id) override;
    float getPoseChange(int id) override;
    void erase(int id) override;
};


#endif //DEPTHEST_MEDIANWINDOWFILTERPOINTMAP_H
