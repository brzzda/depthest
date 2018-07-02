//
// Created by Peter Zborovský on 1.11.2017.
//

#ifndef DEPTHEST_DEPTHESTIMATOR_H
#define DEPTHEST_DEPTHESTIMATOR_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

/**
 * Abstract class that serves for various techniques of depth estimation
 */
class DepthEstimator {
public:
    virtual void addPoint(cv::Point2f point, int id) = 0;
    virtual void deletePoint(int id) = 0;
    virtual void estimateDepth(cv::Mat &out) = 0;
    virtual void addPose(cv::Mat pose) = 0;
    virtual std::vector<cv::Point2f> getFirstViewPoints2D() = 0;
    virtual std::vector<cv::Point2f> getSecondViewPoints2D() = 0;
    virtual void enableEvaluation(bool enabled) = 0;
};

#endif //DEPTHEST_DEPTHESTIMATOR_H
