//
// Created by peetaa on 10.11.2017.
//

#include "Camera.h"
#include "CalibrationLoader.h"
#include "Params.h"

Camera::Camera() {
    CalibrationLoader calibrationLoader(Params::getInstance().getCalibrationFileName());
    CalibrationConfiguration config = calibrationLoader.getConfiguration();

    cameraMatrix = (cv::Mat_<double>(3,3)
         << config.getFx(), 0, config.getCx(),
            0, config.getFy(), config.getCy(),
            0, 0, 1);
    distCoeffs = (cv::Mat_<double>(4,1) << config.getD0(), config.getD1(), config.getD2(), config.getD3());
    cameraWidth = config.getCameraWidth();
    cameraHeight = config.getCameraHeight();

//    cameraMatrix = (cv::Mat_<double>(3,3)
//         << 544.651702, 0, 346.314408,
//            0, 546.917851, 171.445347,
//            0, 0, 1);
//    distCoeffs = (cv::Mat_<double>(4,1) << -0.489389, 0.213254, 0.000654, 0.000895);
//    cameraWidth = 640;
//    cameraHeight = 360;
    size = cv::Size_<int>(cameraWidth, cameraHeight);
}

Camera &Camera::getInstance() {
    static Camera instance;
    return instance;
}

const cv::Mat &Camera::getCameraMatrix() const {
    return cameraMatrix;
}

void Camera::setCameraMatrix(const cv::Mat &cameraMatrix) {
    Camera::cameraMatrix = cameraMatrix;
}

const cv::Mat &Camera::getDistCoeffs() const {
    return distCoeffs;
}

void Camera::setDistCoeffs(const cv::Mat &distCoeffs) {
    Camera::distCoeffs = distCoeffs;
}

const cv::Size &Camera::getSize() const {
    return size;
}

void Camera::setSize(const cv::Size &size) {
    Camera::size = size;
}
