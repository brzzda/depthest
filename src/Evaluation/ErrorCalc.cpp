//
// Created by peetaa on 16.4.2018.
//

#include <EstimatorMaps/Estimators/KalmanFilterEstimator.h>
#include <Evaluation/ErrorCalc.h>
#include <HelperFunctions.h>

ErrorCalc::ErrorCalc() {
    pointCount = 0;
    framePointCount = 0;
    allPointCount = 0;
    totalDistance = 0;
    oneFrameDistance = 0;
    frameCount = 0;
    totalReprojectionError = 0;
    minPointsThresh = 10;
    sparseFrameCount = 0;
    frameDisplacementSum = 0;
}

ErrorCalc::~ErrorCalc() {

}

void ErrorCalc::setReprojValues(cv::Mat _projM1, cv::Mat _projM2, cv::Mat _points1, cv::Mat _points2, cv::Mat _points3D) {

    projM1 = _projM1;
    projM2 = _projM2;
    points1 = _points1;
    points2 = _points2;
    points3D = _points3D;

    cv::Mat pose1 = getCameraPose(projM1);
    cv::Mat pose2 = getCameraPose(projM2);
    frameDisplacement = getDistance(pose1, pose2);
    distRatioCoef = 1 / frameDisplacement;
    frameDisplacementSum += frameDisplacement;
//    ROS_INFO_STREAM("EC dist Ratio Coef: " << distRatioCoef);
//    ROS_INFO_STREAM("EC frame displacement: " << frameDisplacement);
}

void ErrorCalc::calculateReprojError() {
    cv::Mat reprojPts1;
    cv::Mat reprojPts2;


    //ensure same type of 3d points and projection matrices (double or float)
    if (projM1.type() != points3D.type()) {
        points3D.convertTo(points3D, projM1.type());
    }

    //convert tracked points from 2 channel to 1 channel and transpose them so they
    //are ordered in columns
    cv::Mat ptss1[2];
    cv::Mat ptss2[2];
    if(points1.channels() != 1) {
        cv::split(points1, ptss1);
        cv::split(points2, ptss2);
    }
    cv::Mat pts1;
    cv::Mat pts2;
    pts1.push_back(ptss1[0]);
    cv::hconcat(pts1, ptss1[1], pts1);
    pts2.push_back(ptss2[0]);
    cv::hconcat(pts2, ptss2[1], pts2);
    if(pts1.rows != 2) {
        cv::transpose(pts1, pts1);
        cv::transpose(pts2, pts2);
    }


    //calculate reprojected points
    reprojPts1 = projM1 * points3D;
    reprojPts2 = projM2 * points3D;

    cv::Mat rp1;
    cv::Mat rp2;
    //convert homogenuous reprojected points to inhomogenuous coordinates
    makeInhomogenous(reprojPts1, reprojPts1);
    makeInhomogenous(reprojPts2, reprojPts2);


    cv::Mat s1;
    cv::Mat s2;
    if (pts1.type() != reprojPts1.type()) {
        pts1.convertTo(pts1, reprojPts1.type());
        pts2.convertTo(pts2, reprojPts2.type());
    }

    // calculate reprojection error
    // first subtract adequate coordinates, then
    // square the results and finaly add them.
    cv::subtract(reprojPts1, pts1, s1);
    cv::subtract(reprojPts2, pts2, s2);
    cv::Mat sqr1;
    cv::Mat sqr2;
    cv::multiply(s1, s1, sqr1);
    cv::multiply(s2, s2, sqr2);

    cv::Mat dist1 = sqr1.row(0) + sqr1.row(1);
    cv::Mat dist2 = sqr2.row(0) + sqr2.row(1);
    cv::sqrt(dist1,dist1);
    cv::sqrt(dist2,dist2);
    errors = dist2;
//    errors = dist1 + dist2;
//    errors = sqr1.row(0) + sqr1.row(1) + sqr2.row(0) + sqr2.row(1);

    //total reproj error
    cv::Mat res;
    cv::Scalar sum = cv::sum(errors);

    totalReprojectionError += sum[0];
//average reproj error
//    cv::Mat res;
    int count = errors.cols;
//    cv::Scalar sum = cv::sum(errors);
//    std::cout << "aver error " << sum[0] / count << std::endl;
//    cv::reduce(errors, res, 1, CV_REDUCE_SUM);
    oneFrameAverageReprojectionError = static_cast<float>(sum[0] / count);
    averageReprojectionErrors.push_back(oneFrameAverageReprojectionError);


    averageReprojectionErrors.push_back(static_cast<float &&>(getAverageReprojectionError()));
    frameCount++;
}

double ErrorCalc::getReprojectionError() {
    cv::Mat res;
    cv::Scalar sum = cv::sum(errors);
//    std::cout << "error " << sum[0] << std::endl;
    cv::reduce(errors, res, 1, CV_REDUCE_SUM);
    return res.at<double>(0,0);
}

double ErrorCalc::getAverageReprojectionError() {
    return oneFrameAverageReprojectionError;
}

double ErrorCalc::getReprojectionMedianError() {
    cv::Mat sorted;
    cv::sort(errors, sorted, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
    return sorted.at<double>(0, sorted.cols / 2);
}

double ErrorCalc::getDistRatio() {
    if(points3D.cols > 1) {
//        cv::Mat cm = Camera::getInstance().getCameraMatrix();
//        double fx = cm.at<double>(0, 0);
//        double cx = cm.at<double>(0, 2);
//        double fy = cm.at<double>(1, 1);
//        double cy = cm.at<double>(1, 2);
////        outputMatInfo(projM2, "REC proj m");
//        double x = projM2.at<double>(0, 3);
//        double y = projM2.at<double>(1, 3);
//        double camz = projM2.at<double>(2, 3);
        cv::Mat cam = getCameraPose(projM2);
////        std::cout << "REC camx camy" << std::endl;
//        double camx = (x - cx * camz) / fx;
//        double camy = (y - cy * camz) / fy;
//
//        cam.push_back(camx);
//        cam.push_back(camy);
//        cam.push_back(camz);


//        std::cout << "REC cam " << std::endl << cam << std::endl;

        cv::Mat pts;
        makeInhomogenous(points3D, pts);
        pts.convertTo(pts, cam.type());
        cv::Mat d1;
//        std::cout << "REC 1" << std::endl;
        cv::subtract(pts.col(0), cam, d1);
        cv::Mat t1;
//        std::cout << "REC 2" << std::endl;
        cv::transpose(d1, t1);
//        std::cout << "REC d1 t1" << std::endl << d1 << std::endl << t1 << std::endl;
        d1 = t1 * d1;
//        std::cout << " d1 " << std::endl << d1 << std::endl;
//        std::cout << "REC 4" << std::endl;
        double dist1 = d1.at<double>(0, 0);
//        std::cout << "d1 val " << dist1 << std::endl;
//        std::cout << "REC 5" << std::endl;
        cv::subtract(pts.col(1), cam, d1);
//        std::cout << "REC 6" << std::endl;
        cv::transpose(d1, t1);
//        std::cout << "REC 7" << std::endl;
        d1 = t1 * d1;

//        std::cout << "REC 8" << std::endl;
        double dist2 = d1.at<double>(0, 0);
//        std::cout << "d2 val " << dist2 << std::endl;
        return dist1 > dist2 ? dist2 / dist1 : dist1 / dist2;
    } else {
        return -1;
    }
}


double ErrorCalc::getTotalAverageDistance(){
    return totalDistance / allPointCount;
}

double ErrorCalc::getAverageDistance() {
    return averageDistances[averageDistances.size()-1];
}

double ErrorCalc::getMedianDistance() {
    return medianDistances[medianDistances.size()-1];
}

void ErrorCalc::addDistance(float dist) {
//    std::cout << "EC add distanceSum " << dist << std::endl;
//    double dst = dist * distRatioCoef;
//    dist *= distRatioCoef;

    totalDistance+=dist;
    oneFrameDistance+=dist;
//    std::cout << "EC ADD dist " << oneFrameDistance << std::endl;
    allPointCount++;
    framePointCount++;
    oneFrameDistances.push_back(dist);
}

void ErrorCalc::calculateDistanceError() {
//    std::cout << "EC calc dist error " << std::endl;
//    std::cout << "EC total distanceSum " << totalDistance << std::endl;
//    std::cout << "EC all point count " << allPointCount << std::endl;
//    std::cout << "EC one frame distanceSum " << oneFrameDistance << std::endl;
//    std::cout << "EC frame point count " << framePointCount << std::endl;
//    std::cout << "EC total avera"
    averageDistances.push_back(oneFrameDistance / framePointCount);
    auto id = static_cast<int>(framePointCount / 2) + 1;
    std::sort(oneFrameDistances.begin(), oneFrameDistances.end());
    medianDistances.push_back(oneFrameDistances[id]);
    allDistances.push_back(oneFrameDistance);
    oneFrameDistance = 0;
    pointsCount.push_back(framePointCount);
    if(framePointCount < minPointsThresh) {
        sparseFrameCount++;
    }
    framePointCount = 0;
    oneFrameDistances.clear();
}

void ErrorCalc::scaleErrors() {
    std::vector<float> dists;

//    double minDist =
}

float ErrorCalc::getCurrentFramePoseChangeError() {
    return allDistances[allDistances.size()-1];
}

float ErrorCalc::getTotalDistance() {
    return static_cast<float>(totalDistance);
}

float ErrorCalc::getTotalReprojectionError() {
    return static_cast<float>(totalReprojectionError);
}

float ErrorCalc::getTotalAverageReprojectionError() {
//    return static_cast<float>(totalReprojectionError / frameCount);
    return static_cast<float>(totalReprojectionError / allPointCount);
}

float ErrorCalc::getMedianOfAverageReprojectionErrors() {
    int id = static_cast<int>(averageReprojectionErrors.size() / 2);
    std::sort(averageReprojectionErrors.begin(), averageReprojectionErrors.end());
    return averageReprojectionErrors[id];
}

float ErrorCalc::getMedianOfAverageDistances() {
    int id = static_cast<int>(averageDistances.size() / 2);
    std::sort(averageDistances.begin(), averageDistances.end());
    return averageDistances[id];
}

float ErrorCalc::getAveragePointCount() {
    return allPointCount / frameCount;
}

ulong ErrorCalc::getTotalPointCount() {
    return allPointCount;
}

int ErrorCalc::getSparfeFrameCount() {
    return sparseFrameCount;
}

ulong ErrorCalc::getFrameCount() {
    return frameCount;
}

double ErrorCalc::getAverageFrameDisplacement() {
    return frameDisplacementSum / frameCount;
}

double ErrorCalc::getCurrentFrameDisplacement() {
    return frameDisplacement;
}
