//
// Created by peetaa on 27.4.2018.
//

#include "PoseChangeFilter.h"
#include <HelperFunctions.h>

PoseChangeFilter::PoseChangeFilter() {
    distLastAverage = 0;
    distanceSum = 0;
    distanceLastVar = 0;
    count = 0;
    first = true;

}

void PoseChangeFilter::addPose(cv::Mat pose) {
    count++;
    if (first) {
        distance = 0;
        distAverage = 0;
        first = false;
    } else {
        distance = getDistance(lastPose, pose);
//        distAverage = distanceSum / count - 1;
    }
    distanceSum += distance;
    pose.copyTo(lastPose);

    distAverage = (distance + ((count - 1 )* distLastAverage ))/count;
    distanceVar = distanceLastVar + ((distance - distLastAverage) * (distance - distAverage));

    distanceLastVar = distanceVar;
    distLastAverage = distAverage;


}

float PoseChangeFilter::getVariance() {
    return static_cast<float>(sqrt(distanceVar / count));
}

