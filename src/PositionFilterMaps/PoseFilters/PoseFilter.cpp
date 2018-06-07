//
// Created by peetaa on 27.4.2018.
//

#include <HelperFunctions.h>
#include <PositionFilterMaps/PoseFilters/PoseFilter.h>

PoseFilter::PoseFilter() {
    xAverage = 0;
    yAverage = 0;
    zAverage = 0;
    count = 0;
    xLastAverage = 0;
    yLastAverage = 0;
    zLastAverage = 0;
    xLastVar = 0;
    yLastVar = 0;
    zLastVar = 0;
}

void PoseFilter::addPose(cv::Mat pose) {
    count++;
    x = pose.at<float>(0, 0);
    y = pose.at<float>(1, 0);
    z = pose.at<float>(2, 0);

    pose.copyTo(lastPose);

    xAverage = (x + (xLastAverage * (count - 1)))/count;
    yAverage = (y + (yLastAverage * (count - 1)))/count;
    zAverage = (z + (zLastAverage * (count - 1)))/count;

    xVar = xLastVar + (x - xLastAverage) * (x - xAverage);
    poseVar = xVar;
    yVar = yLastVar + (y - yLastAverage) * (y - yAverage);
    poseVar = poseVar > yVar ? poseVar : yVar;
    zVar = zLastVar + (z - zLastAverage) * (z - zAverage);
    poseVar = poseVar > zVar ? poseVar : zVar;

    xLastAverage = xAverage;
    yLastAverage = yAverage;
    zLastAverage = zAverage;
    xLastVar = xVar;
    yLastVar = yVar;
    zLastVar = zVar;

}

float PoseFilter::getVariance() {
    return static_cast<float>(sqrt(poseVar / count));
}
