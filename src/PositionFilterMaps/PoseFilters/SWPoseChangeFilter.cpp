//
// Created by peetaa on 27.4.2018.
//

#include <Params.h>
#include <HelperFunctions.h>
#include "SWPoseChangeFilter.h"

SWPoseChangeFilter::SWPoseChangeFilter() {
    windowSize = Params::getInstance().getMedianFilterWindowSize();
    count = 0;
    distanceSum = 0;
    first = true;
}

void SWPoseChangeFilter::addPose(cv::Mat pose) {
    count++;
    float distance;
    if(first) {
        distance = 0;
        count--;
    } else {

        distance = getDistance(pose, lastPose);
        distances.push_back(distance);
    }
    pose.copyTo(lastPose);
    distanceSum += distance;
    if(count > windowSize) {
        distanceSum -= distances.front();
        distances.erase(distances.begin());
        count--;
    }
    if(first) {
        var = 0;
        first = false;
    } else {
        var = calculateVariance();
    }
}

float SWPoseChangeFilter::getVariance() {
    return var;
}

float SWPoseChangeFilter::calculateVariance() {
    float mean = distanceSum / count;
    float varSum = 0;
    for (int i = 0; i < count; i++) {
        varSum += (distances[i] - mean) * (distances[i] - mean);
    }
//    std::cout <<"wpcv variance " <<  sqrt(varSum / count);
    return static_cast<float>(sqrt(varSum / count));
}
