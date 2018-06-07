//
// Created by peetaa on 23.4.2018.
//

#ifndef DEPTHEST_KALMANFILTERPOINTMAP_H
#define DEPTHEST_KALMANFILTERPOINTMAP_H

#include "EstimatorMaps/Estimators/KalmanFilterEstimator.h"
#include "EstimatorPointMap.h"

/**
 * wrapper for map of Kalman filter Position estimator
 * each point is stored, accessed or erased  here.
 */
class KalmanFilterEstimatorPointMap : public EstimatorPointMap {
private:
    std::map<int, KalmanFilterEstimator> points3D;
public:
    void update(int id, cv::Mat point) override;
    cv::Mat getEstimate(int id) override;
    float getPoseChange(int id) override;
    void erase(int id) override;
};


#endif //DEPTHEST_KALMANFILTERPOINTMAP_H
