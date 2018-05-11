//
// Created by peetaa on 22.4.2018.
//

#include <HelperFunctions.h>
#include "DummyEstimator.h"

DummyEstimator::DummyEstimator() {
    pose = cv::Mat(3,1, CV_32FC1);
    pose = 0;
    first = true;
}

DummyEstimator::~DummyEstimator() {

}

void DummyEstimator::update(cv::Mat point) {
    distance = getDistance(pose, point);
    point.copyTo(pose);
}

cv::Mat DummyEstimator::getEstimate() {
    return pose;
}

float DummyEstimator::getPoseChange() {
    if (first) {
        first = false;
        return 0;
    }
    return distance;
}
