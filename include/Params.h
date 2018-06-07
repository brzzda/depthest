//
// Created by peetaa on 24.4.2018.
//

#ifndef DEPTHEST_PARAMS_H
#define DEPTHEST_PARAMS_H
#include <string>
/**
 * singleton class to store and access global parameters
 */
class Params {
private:
    bool enableMeasurement;
    std::string estimator;
    float kfProcessNoiseCov;
    float kfMeasNoiseCov;
    int triangSlidingWindowSize;
    int medianFilterSlidingWindowSize;
    std::string calibrationFileName;
    float poseVarianceThres;
    float poseChangeVarianceThresh;
    std::string filter;
    std::string imageTopic;
    std::string undistImateTopic;
    std::string odometryTopic;
    std::string featuresTopic;

    Params();
    Params(Params const&);             // Don't Implement
    void operator=(Params const&);     // Don't implement

public:
    static Params& getInstance();
    const bool &getEnabledMeasurement() const;
    const std::string getEstimatorName() const;
    const float getKFProcessNoise() const;
    const float getKFMeasnoise() const;
    const int getTriangWindowSize() const;
    const int getMedianFilterWindowSize() const;
    const float getPoseVarianceThresth() const;
    const float getPoseChangeVarianceThresth() const;
    const std::string getFilterName() const;
    const std::string getCalibrationFileName() const;
    const std::string &getImageTopic() const;
    const std::string &getUndistImateTopic() const;

    const std::string &getFeaturesTopic() const;

    const std::string &getOdometryTopic() const;
};


#endif //DEPTHEST_PARAMS_H
