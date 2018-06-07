//
// Created by peetaa on 22.4.2018.
//

#include <iostream>
#include <Params.h>
#include <EstimatorMaps/Estimators/MedianSWEstimator.h>
#include <HelperFunctions.h>


MedianSWEstimator::MedianSWEstimator() {
//    std::cout << "pt3D create " << std::endl;
    countAvg = 0;
    countMed = 0;
    pose = cv::Mat(3,1, CV_32FC1);
    pose = 0;
    first = true;
    slidingWindowSize = Params::getInstance().getMedianFilterWindowSize();
}

MedianSWEstimator::~MedianSWEstimator() {
    Z.clear();
    Y.clear();
    X.clear();
}

void MedianSWEstimator::update(cv::Mat pt) {
//    std::cout << "pt3D update" << std::endl;
//    outputMatInfo(pt, "pt3d inserted point");
    X.push_back(pt.at<float>(0,0));
//    std::cout << "111" << std::endl;
    Y.push_back(pt.at<float>(1,0));
//    std::cout << "222" << std::endl;
    Z.push_back(pt.at<float>(2,0));
//    std::cout << "333" << std::endl;

//    std::cout << "P3D inserted point" << std::endl << pt << std::endl;

    if(countMed > slidingWindowSize) {
//        std::cout << "P3D 0 " << X.at(0) << std::endl;
//        std::cout << "P3D end " << X.at(X.size() - 1) << std::endl;

//        if (countAvg % 2 == 1) {
//            std::cout << "P3D begin " << std::endl;
            X.erase(X.begin());
            Y.erase(Y.begin());
            Z.erase(Z.begin());
//        } else {
////            std::cout << "P3D end " << std::endl;
//            X.erase(X.end()-1);
//            Y.erase(Y.end()-1);
//            Z.erase(Z.end()-1);
//        }
        countMed--;
    }
    estimatedPose = calculateEstimatedPose();
    poseChange = getDistance(pose, estimatedPose);
    countAvg++;
    countMed++;
    estimatedPose.copyTo(pose);
//    std::cout << "pt3D update FINITO" << std::endl;
}

cv::Mat MedianSWEstimator::getEstimate() {
    return estimatedPose;
}

float MedianSWEstimator::getPoseChange() {
    if (first) {
        first = false;
        return 0;
    }
    return static_cast<float>(poseChange);
}

cv::Mat MedianSWEstimator::calculateEstimatedPose() {
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
