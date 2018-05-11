//
// Created by peetaa on 27.4.2018.
//

#include <Params.h>
#include "SWPoseFilter.h"

SWPoseFilter::SWPoseFilter() {
    count = 0;
    xSum = 0;
    ySum = 0;
    zSum = 0;
    windowSize = Params::getInstance().getMedianFilterWindowSize();
}

void SWPoseFilter::addPose(cv::Mat pose) {
    x.push_back(pose.at<float>(0, 0));
    y.push_back(pose.at<float>(1, 0));
    z.push_back(pose.at<float>(2, 0));
    xSum += pose.at<float>(0,0);
    ySum += pose.at<float>(1,0);
    zSum += pose.at<float>(2,0);
    count++;
    if (count > windowSize) {

        xSum -= x.front();
        ySum -= y.front();
        zSum -= z.front();
        x.erase(x.begin());
        y.erase(y.begin());
        z.erase(z.begin());
        count--;
    }

    var = calculateVariance();
}

float SWPoseFilter::getVariance() {
    return var;
}

float SWPoseFilter::calculateVariance() {
    float xMean = xSum / count;
    float yMean = ySum / count;
    float zMean = zSum / count;
    float xVarSum = 0;
    float yVarSum = 0;
    float zVarSum = 0;
    for (int i = 0; i < count; i++) {
        xVarSum += (x[i] - xMean) * (x[i] - xMean);
        yVarSum += (y[i] - yMean) * (y[i] - yMean);
        zVarSum += (z[i] - zMean) * (z[i] - zMean);
    }
    xVar = static_cast<float>(sqrt(xVarSum / count));
    yVar = static_cast<float>(sqrt(yVarSum / count));
    zVar = static_cast<float>(sqrt(zVarSum / count));
    float maxVar = xVar;
    if (yVar > maxVar)
        maxVar = yVar;
    if (zVar > maxVar)
        maxVar = zVar;
    return maxVar;
}
