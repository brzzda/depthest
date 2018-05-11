//
// Created by peetaa on 16.4.2018.
//

#ifndef DEPTHEST_REPROJERRORCALC_H
#define DEPTHEST_REPROJERRORCALC_H

#include <opencv2/opencv.hpp>
#include <EstimatorMaps/Estimators/PositionEstimator.h>

/**
 * only for evaluation purposes
 * reprojection error and model stability error is calculated here
 * TODO - optimise
 */
class ErrorCalc {
private:
    cv::Mat errors;
    std::vector<double> errors2;
    cv::Mat projM1;
    cv::Mat projM2;
    cv::Mat points1;
    cv::Mat points2;
    cv::Mat points3D;

    double totalReprojectionError;
    std::vector<double> reprojectionErrors;
    std::vector<ulong> pointsCount;

    double totalDistance;

    int minPointsThresh;
    int sparseFrameCount;
    ulong frameCount;
    ulong pointCount;
    float oneFrameDistance;
    ulong allPointCount;
    ulong framePointCount;
    float oneFrameAverageReprojectionError;
//    ulong lastFramePointCount;
    std::vector<float> oneFrameDistances;
    std::vector<float> averageReprojectionErrors;
    std::vector<float> allDistances;
    std::vector<float> averageDistances;
    std::vector<float> medianDistances;
    std::map<int, std::vector<float>> distancesByPointId;
    double distanceToErrorRatio;
    double frameDisplacement;
    double frameDisplacementSum;
    double distRatioCoef;
    void scaleErrors();
public:
    ErrorCalc();
    ~ErrorCalc();
    void setReprojValues(cv::Mat _projM1, cv::Mat _projM2, cv::Mat _points1, cv::Mat _points2, cv::Mat _points3D);
//    void setPoseChangeValues(std::map<int, KalmanFilterEstimator*> *points, std::vector<int> ids);
    void addDistance(float dist);
    double getAverageDistance();
    double getMedianDistance();
    float getCurrentFramePoseChangeError();
    double getTotalAverageDistance();
    float getTotalDistance();
    void calculateReprojError();
    void calculateDistanceError();
    float getTotalReprojectionError();
    float getTotalAverageReprojectionError();
    double getAverageReprojectionError();
    double getReprojectionError();
    double getReprojectionMedianError();
    double getDistRatio();
    float getMedianOfAverageReprojectionErrors();
    float getMedianOfAverageDistances();
    float getAveragePointCount();
    ulong getTotalPointCount();
    int getSparfeFrameCount();
    ulong getFrameCount();
    double getAverageFrameDisplacement();
    double getCurrentFrameDisplacement();
};


#endif //DEPTHEST_REPROJERRORCALC_H
