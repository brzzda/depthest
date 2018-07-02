//
// Created by peetaa on 10.11.2017.
//

#ifndef DEPTHEST_CAMERA_H
#define DEPTHEST_CAMERA_H


#include <opencv2/opencv.hpp>

/**
 * Intrinsic camera parameters
 */
class Camera {
private:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    int cameraWidth;
    int cameraHeight;
    cv::Size size;

    Camera();
    Camera(Camera const&);             // Don't Implement
    void operator=(Camera const&);     // Don't implement

public:
    static Camera& getInstance();

    const cv::Size &getSize() const;

    void setSize(const cv::Size &size);


    const cv::Mat &getCameraMatrix() const;

    void setCameraMatrix(const cv::Mat &cameraMatrix);

    const cv::Mat &getDistCoeffs() const;

    void setDistCoeffs(const cv::Mat &distCoeffs);

};


#endif //DEPTHEST_CAMERA_H
