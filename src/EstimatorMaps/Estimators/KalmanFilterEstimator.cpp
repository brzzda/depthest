//
// Created by peetaa on 22.4.2018.
//

#include <HelperFunctions.h>
#include <Params.h>
#include "KalmanFilterEstimator.h"

KalmanFilterEstimator::KalmanFilterEstimator() {
    KF.init(3, 3, 0, CV_32F);
//    count = 0;
//    KF.transitionMatrix = (cv::Mat_<float>(3, 3) << 1, 1, 0, 1);
    setIdentity(KF.transitionMatrix);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, cv::Scalar::all(Params::getInstance().getKFProcessNoise()));
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(Params::getInstance().getKFMeasnoise()));//(1e-1));
//    KF.measurementNoiseCov.at<float>(2,2) = 30;
    setIdentity(KF.errorCovPost, cv::Scalar::all(1));
//    setIdentity(KF.errorCovPre, cv::Scalar::all(0.1));
    pose = cv::Mat(3,1,CV_32FC1);
    pose = 0;
    first = true;
}

KalmanFilterEstimator::~KalmanFilterEstimator() {

}

void KalmanFilterEstimator::update(cv::Mat point) {
    // First predict, to update the internal statePre variable
//    std::cout << "KF predict" << std::endl;
    if (first) {
        point.copyTo(KF.statePre);
    }
    cv::Mat prediction = KF.predict();

    // The update phase
    estimatedPose = KF.correct(point);
//    count++;
//    if(count > 30) {
//        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(4));//(1e-1));
//    }
    poseChange = getDistance(pose, estimatedPose);
    estimatedPose.copyTo(pose);
//    point.copyTo(estimatedPose);
}

cv::Mat KalmanFilterEstimator::getEstimate() {
    return estimatedPose;
}

void KalmanFilterEstimator::init(cv::Mat initialState) {
//    count = 0;
    first = true;
    KF.init(3, 3, 0, CV_32F);

    setIdentity(KF.transitionMatrix);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));
    setIdentity(KF.measurementNoiseCov, cv::Scalar::all(3));//(1e-1));
    setIdentity(KF.errorCovPost, cv::Scalar::all(1));
//    state = cv::Mat(initialState);
}

float KalmanFilterEstimator::getPoseChange() {
    if(first) {
        first = false;
        return 0;
    }
    return poseChange;
}
