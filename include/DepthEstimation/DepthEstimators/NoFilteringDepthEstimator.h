//
// Created by peetaa on 27.4.2018.
//

#ifndef DEPTHEST_SIMPLEDEPTHESTIMATOR2_H
#define DEPTHEST_SIMPLEDEPTHESTIMATOR2_H



//
// Created by peetaa on 18.11.2017.
//

//#include "ros/ros.h"
#include <EstimatorMaps/EstimatorPointMapFactory.h>
#include <Params.h>
#include "DepthEstimation/DepthEstimators/DepthEstimator.h"
#include "Evaluation/ErrorCalc.h"
#include "EstimatorMaps/Estimators/MedianEstimator.h"
#include "EstimatorMaps/Estimators/MedianSWEstimator.h"
#include "EstimatorMaps/Estimators/KalmanFilterEstimator.h"
/**
 * Depth estimation without usage of filters
 */
class NoFilteringDepthEstimator : public DepthEstimator {
private:
    std::queue<cv::Mat> poseBuffer;
    std::queue<cv::Mat> pointsBuffer;

    cv::Mat firstPose;
    cv::Mat firstPoints;

    int POINT_FILTER_TYPE;
    const int displacement = 20u;
    cv::Mat lastPose;
    cv::Mat lastPoints;
    cv::Mat lastProjMatrix;
    cv::Mat cameraMatrix;

    Params& params;

    ros::NodeHandle nh;

    std::queue<cv::Mat> poses;
    // map of points history / int - id of point que. que - point appearance history
    std::map<int, std::queue<cv::Point2f>> points;

    EstimatorPointMapFactory pointMapFactory;
    EstimatorPointMap* points3D;
//    std::map<int, MedianEstimator> points3D;
//    std::map<int, MedianSWEstimator> points3D;
//    std::map<int, DummyEstimator> points3D;
//    std::map<int, KalmanFilterEstimator> points3D;
//    std::map<int, PositionEstimator> points3D;
    std::map<int, cv::Mat> lastPoints3D;

    std::vector<cv::Point2f> secondViewPoints2D;
    std::vector<cv::Point2f> firstViewPoints2D;
    std::vector<cv::Point2f> secondViewPoints2DOut;
    std::vector<cv::Point2f> firstViewPoints2DOut;
    std::vector<int> ids;

    int frameDisplacement;
    int type_;
//    Camera& camera;

    bool evaluationEnabled;
    ErrorCalc evaluator;
public:
    NoFilteringDepthEstimator();
    ~NoFilteringDepthEstimator();
    void addPoint(cv::Point2f point, int id) override;
    void deletePoint(int id) override;
    void estimateDepth(cv::Mat &out) override;
    void addPose(cv::Mat pose) override;
    void evaluate(cv::Mat proj1, cv::Mat proj2, cv::Mat pts1, cv::Mat pts2, cv::Mat pts3D);
    std::vector<cv::Point2f> getFirstViewPoints2D() override;
    std::vector<cv::Point2f> getSecondViewPoints2D() override;
    void update3DPoints(cv::Mat in, cv::Mat &out);
    void enableEvaluation(bool enabled) override;
};



#endif //DEPTHEST_SIMPLEDEPTHESTIMATOR2_H
