//
// Created by Peter Zborovský on 21.4.2018.
//

#include <iostream>
#include "MedianEstimator.h"
#include "HelperFunctions.h"


MedianEstimator::MedianEstimator() {
    countMed = 0;
    pose = cv::Mat(3,1, CV_32FC1);
    pose = 0;
    first = true;
}

MedianEstimator::~MedianEstimator() {
    Z.clear();
    Y.clear();
    X.clear();
}

void MedianEstimator::update(cv::Mat point) {

    X.push_back(point.at<float>(0,0));
    Y.push_back(point.at<float>(1,0));
    Z.push_back(point.at<float>(2,0));

    estimatedPose = estimatePose();
    distance = getDistance(pose, estimatedPose);
    estimatedPose.copyTo(pose);

    countMed++;
}

cv::Mat MedianEstimator::getEstimate() {
    return estimatedPose;
}

float MedianEstimator::getPoseChange() {
    if (first) {
        first = false;
        return 0;
    }
    return distance;
}

cv::Mat MedianEstimator::estimatePose() {
    cv::Mat res;
    std::vector<float> x;
    std::vector<float> z;
    std::vector<float> y;
    ulong id = countMed / 2;

    cv::sort(X, x, CV_SORT_ASCENDING);
    cv::sort(Y, y, CV_SORT_ASCENDING);
    cv::sort(Z, z, CV_SORT_ASCENDING);

    res.push_back(x[id]);
    res.push_back(y[id]);
    res.push_back(z[id]);
    return res;
}
