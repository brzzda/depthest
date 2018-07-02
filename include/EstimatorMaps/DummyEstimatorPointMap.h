//
// Created by peetaa on 23.4.2018.
//

#ifndef DEPTHEST_DUMMYFILTERPOINTMAP_H
#define DEPTHEST_DUMMYFILTERPOINTMAP_H

#include "EstimatorMaps/Estimators/DummyEstimator.h"
#include "EstimatorPointMap.h"

/**
 * wrapper for map of dummy Position estimator
 * each point is stored, accessed or erased  here.
 */
class DummyEstimatorPointMap : public EstimatorPointMap {
private:
    std::map<int, DummyEstimator> points3D;
public:
    void update(int id, cv::Mat point) override;
    cv::Mat getEstimate(int id) override;
    float getPoseChange(int id) override;
    void erase(int id) override;
};


#endif //DEPTHEST_DUMMYFILTERPOINTMAP_H
