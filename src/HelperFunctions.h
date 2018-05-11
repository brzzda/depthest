//
// Created by peetaa on 11.12.2017.
//

#ifndef DEPTHEST_HELPERFUNCTIONS_H
#define DEPTHEST_HELPERFUNCTIONS_H


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <depthest/ObjectsStamped.h>
#include <depthest/FlowArrayStamped.h>
#include <depthest/FlowArrayStampedAged.h>
#include <depthest/PointsStamped.h>
#include <depthest/Point2D.h>

#include <tf2_msgs/TFMessage.h>
#include <eigen3/Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.h>
#include <opencv2/core/eigen.hpp>
#include <Camera.h>

/*
 * Various static functions and constants
 */

inline static void fromPointsStamped2cvMat(std::vector<geometry_msgs::Point> in, cv::Mat& out) {
    out = cv::Mat();
    out.push_back(in[0].x);
    out.push_back(in[0].y);
    out.push_back(in[0].z);
//    std::cout << " from msg to pts3d" << std::endl;
    for (int i = 1; i < in.size(); i++) {
        cv::Point3d pt;
        pt.x = in[i].x;
        pt.y = in[i].y;
        pt.z = in[i].z;
        cv::Mat col(pt);
//        ROS_INFO_STREAM("from points to mat: pt size " << col.rows << " " << col.cols);
        cv::hconcat(out,col,out);
    }
}


static int SMOOTHING_WINDOW_SIZE = 70;
static int SIMPLE_DEPTH_ESTIMATOR = 1;
static int FILTERED_DEPTH_ESTIMATOR = 0;
static int OBJECT_DEPTH_ESTIMATOR = 3;
static int RECTIFIED_DEPTH_ESTIMATOR = 4;

static std::string KALMAN_POINT_FILTER = "kalman";
static std::string AVERAGE_POINT_FILTER = "average";
static std::string MEDIAN_POINT_FILTER = "median";
static std::string MEDIAN_WINDOW_PONT_FILTER = "median_window";
static std::string DUMMY_POINT_FILTER = "dummy";


static std::string POSE_VARIANCE = "pose_var";
static std::string POSE_CHANGE_VARIANCE = "pose_change_var";
static std::string NONE = "none";

inline static int getFrameDisplacement() {
    return 13;
}

inline static cv::Mat getCameraPose(cv::Mat projectionMatrix) {
    cv::Mat cm = Camera::getInstance().getCameraMatrix();
    double fx = cm.at<double>(0, 0);
    double cx = cm.at<double>(0, 2);
    double fy = cm.at<double>(1, 1);
    double cy = cm.at<double>(1, 2);
//        outputMatInfo(projM2, "REC proj m");
    double x = projectionMatrix.at<double>(0, 3);
    double y = projectionMatrix.at<double>(1, 3);
    double camz = projectionMatrix.at<double>(2, 3);
    cv::Mat cam;
//        std::cout << "REC camx camy" << std::endl;
    double camx = (x - cx * camz) / fx;
    double camy = (y - cy * camz) / fy;

    cam.push_back(camx);
    cam.push_back(camy);
    cam.push_back(camz);
    return cam;
}

inline static cv::Mat calculate2DDistances(cv::Mat pts1, cv::Mat pts2, cv::Mat &out) {
    cv::Mat s;
    cv::subtract(pts1, pts2, s);
    cv::Mat sqr;
    cv::multiply(s, s, sqr);

    out = cv::Mat();
    out= sqr.row(0) + sqr.row(1);
    cv::sqrt(out, out);
}

inline static void fromStdPose2cvMat(geometry_msgs::Pose in, cv::Mat& out) {
    //from geometry_msg::Pose to Eigen Matrix to OpenCV Mat - most likely it can be done in better way
    Eigen::Affine3d outt;
    tf2::fromMsg(in, outt);
//    ROS_INFO_STREAM("rotx eigen matrix of pose: " << std::endl << out);
    cv::eigen2cv(outt.matrix(),out);
//    ROS_INFO_STREAM("out size and type " << out.size << " , " << out.type());
//    ROS_INFO_STREAM("rotx cv matrix of pose: " << std::endl << out);
}

//inline static void fromCvMat2stdPose(cv::Mat in, geometry_msgs::Pose &out) {
//    Eigen::Affine3d outt;
//    cv::cv2eigen(in, outt);
//    out = tf2::toMsg(outt);
//}

inline static void fromCvMat2pointsMsg(cv::Mat in, geometry_msgs::Point &out) {
    Eigen::Vector3d point;
    cv::cv2eigen(in,point);
    out = tf2::toMsg(point);
}

inline static void fromCvMat2points2d(cv::Mat in, std::vector<cv::Point2d> &out) {
    if (in.rows > 3) return;
    if (in.rows < 2) return;
    if (in.rows == 3) {
        for (int i = 0; i < in.cols; i++) {
            cv::Point2d pt;
            pt.x = in.at<double>(0,i) / in.at<double>(2, i);
            pt.y = in.at<double>(1,i) / in.at<double>(2, i);
            out.push_back(pt);
        }
    } else {
        for (int i = 0; i < in.cols; i++) {
            cv::Point2d pt;
            pt.x = in.at<double>(0, i);
            pt.y = in.at<double>(1, i);
            out.push_back(pt);
        }
    }
}

inline static float getDistance(cv::Mat pt1, cv::Mat pt2) {
    if(pt1.size != pt2.size) {
        std::cout << "GET DISTANCE - matrices must be of same size" << std::endl;
        return 0;
    }
//    std::cout << "HF GET DISTANCE " << std::endl;
//    std::cout << "HF pt1 " << std::endl << pt1 << std::endl;
//    std::cout << "HF pt2 " << std::endl << pt2 << std::endl;
    if(pt1.type() != 5)
        pt1.convertTo(pt1, 5);
    if(pt2.type() != 5)
        pt2.convertTo(pt2, 5);

    cv::Mat s;
    cv::subtract(pt1, pt2, s);
    cv::Mat ss;
    cv::transpose(s, ss);
    s = ss * s;
    cv::sqrt(s, s);
    return s.at<float>(0,0);
}

inline static void makeInhomogenous(cv::Mat in, cv::Mat& out){
    if(in.rows == 3) {
        cv::Mat res(2, in.cols, in.type());

        cv::divide(in.row(0), in.row(2), res.row(0), 1, in.type());//CV_32F);
        cv::divide(in.row(1), in.row(2), res.row(1), 1, in.type());//CV_32F);
        out = res;
    } else if (in.rows == 4) {
        cv::Mat res(3, in.cols, in.type());

        cv::divide(in.row(0), in.row(3), res.row(0), 1, in.type());//CV_32F);
        cv::divide(in.row(1), in.row(3), res.row(1), 1, in.type());//CV_32F);
        cv::divide(in.row(2), in.row(3), res.row(2), 1, in.type());//CV_32F);
        out = res;
    } else
        return;
}

inline static void getRotationAndTranslation(cv::Mat pose1, cv::Mat pose2, cv::Mat& R, cv::Mat& T) {
    cv::Mat r(3,3,CV_64FC1);
    cv::Mat t(3,1,CV_64FC1);
//    std::cout << "p1 " << std::endl << pose1 << std::endl;
    cv::Mat r1 = pose1(cv::Rect(0,0,3,3));
    cv::Mat r2 = pose2(cv::Rect(0,0,3,3));

//    std::cout << "r2 " << std::endl << r2 << std::endl;
    cv::Mat t1 = pose1(cv::Rect(3,0,1,3));
    cv::Mat t2 = pose2(cv::Rect(3,0,1,3));
//    std::cout << "t1 " << std::endl << t1 << std::endl;
//    std::cout << "r2 inv " << std::endl << r2.inv() << std::endl;
    cv::transpose(r1, r1);
//    std::cout << "r2 tr " << std::endl << r2 << std::endl;
    R = r1*r2;
    T = t2 - t1;
//    std::cout << "all ok " << std::endl;
}

inline static void fromCvMat2points2f(cv::Mat in, std::vector<cv::Point2f> &out) {
    if (in.rows > 3) return;
    if (in.rows < 2) return;
    if (in.rows == 3) {
        for (int i = 0; i < in.cols; i++) {
            cv::Point2f pt;
            pt.x = in.at<float>(0,i) / in.at<float>(2, i);
            pt.y = in.at<float>(1,i) / in.at<float>(2, i);
            out.push_back(pt);
        }
    } else {
        for (int i = 0; i < in.cols; i++) {
            cv::Point2f pt;
            pt.x = in.at<float>(0, i);
            pt.y = in.at<float>(1, i);
            out.push_back(pt);
        }
    }
}

inline static void fromCvPoints2f2Points2D(std::vector<cv::Point2f> in, std::vector<depthest::Point2D> &out) {
    unsigned long len = in.size();
    out.clear();
    depthest::Point2D pt;
    for (int i = 0; i < len; i++) {
        pt.x = in[i].x;
        pt.y = in[i].y;
        out.push_back(pt);
    }
}

inline static void projectPoints(cv::Mat projMat, cv::Mat points3D, cv::Mat& projectedPts) {

    projectedPts = projMat * points3D;
}

inline static void makeColumnLikeMat(cv::Mat in, cv::Mat &out) {
    switch(in.channels()) {
        case 1:
            if(in.cols <= 3 && in.rows > 3)
                cv::transpose(in, out);
            break;
        case 2:
            if (in.rows > 2) {
                cv::Mat pts[2];
                cv::split(in, pts);
                out = cv::Mat();
                out.push_back(pts[0]);
                cv::hconcat(out, pts[1], out);
                cv::transpose(out, out);
            }
            break;
        case 3:
            if (in.rows > 3) {
                cv::Mat ptss[3];
                cv::split(in, ptss);
                out = cv::Mat();
                out.push_back(ptss[0]);
                cv::hconcat(out, ptss[1], out);
                cv::hconcat(out, ptss[2], out);
                cv::transpose(out, out);
            }
            break;
    }
}

inline static void getRectifiedCoordinates2(cv::Mat mapx, cv::Mat mapy, std::vector<cv::Point2f> in, std::vector<cv::Point2f>& out) {
    out.clear();
    for (int i = 0; i < in.size(); i++) {
        cv::Point2f pt;
        pt.x = mapx.at<float>(static_cast<int>(in[i].x), static_cast<int>(in[i].y));
        pt.y = mapy.at<float>(static_cast<int>(in[i].x), static_cast<int>(in[i].y));
        out.push_back(pt);
    }
}

inline static void getRectifiedCoordinates(cv::Mat R, cv::Mat P, std::vector<cv::Point2f> in, std::vector<cv::Point2f>& out) {
    cv::Mat RT;
    cv::transpose(R, RT);
    cv::Mat CM = Camera::getInstance().getCameraMatrix();
    out.clear();
//    std::cout << "HF GRC 1" << std::endl;
    for (int i = 0; i < in.size(); i++) {
        cv::Mat X;
//        std::cout << "////////////////////////" << std::endl;
//        std::cout << "HF pt in " << in[i] << std::endl;
//        std::cout << "HF P " << std::endl << P << std::endl;
        P.convertTo(P,CV_32FC1);
        float x = (in[i].x - P.at<float>(0, 2)) / P.at<float>(0, 0);
        float y = (in[i].y - P.at<float>(2, 2)) / P.at<float>(1, 1);
//        std::cout << "HF x " << x << "  y " << y << std::endl;
//        std::cout << "HF GRC 3" << std::endl;
        X.push_back(x);
//        std::cout << "HF GRC 31" << std::endl;
        X.push_back(y);
//        std::cout << "HF GRC 32" << std::endl;
        double w = 1.0;
        X.push_back(w);
//        std::cout << "HF converted divided X " << X <<std::endl;
        X.convertTo(X, RT.type());
        cv::Mat XX = RT * X;
//        std::cout << "HF rotated XX " << std::endl <<  XX <<std::endl;
//        std::cout << "HF GRC 5" << std::endl;
        makeInhomogenous(XX, XX);
//        std::cout << "HF XX inhom " << std::endl <<  XX <<std::endl;
//        std::cout << "HF GRC 6" << std::endl;
        cv::Point2d outt;
        outt.x = XX.at<double>(0, 0) * CM.at<double>(0, 0) + CM.at<double>(0, 2);
        outt.y = XX.at<double>(1, 0) * CM.at<double>(1, 1) + CM.at<double>(1, 2);
//        std::cout << "HF GRC 7" << std::endl;
//        std::cout << "HF pt out " << outt <<std::endl;
        out.push_back(outt);
    }
}

inline static void fromCvMat2pointsMsg(cv::Mat in, std::vector<geometry_msgs::Point> &out) {
    geometry_msgs::Point point;
    out.clear();
//    std::cout << "HF from cv mat 2 points msg " << std::endl << in << std::endl << std::endl;
    if(in.type() == 5) {
        for (int i = 0; i < in.cols; i++) {
            point.x = in.at<float>(0, i);
            point.y = in.at<float>(1, i);
            point.z = in.at<float>(2, i);
//            std::cout << point << std::endl;
            out.push_back(point);
        }
    } else if (in.type() == 6) {
        for (int i = 0; i < in.cols; i++) {
            point.x = in.at<double>(0, i);
            point.y = in.at<double>(1, i);
            point.z = in.at<double>(2, i);
//            std::cout << point << std::endl;
            out.push_back(point);
        }
    }
//    Eigen::Vector3d pt;
//    for(int i = 0; i < in.cols; i++) {
//        cv::cv2eigen(in.col(i),pt);
//        point = tf2::toMsg(pt);
//        out.push_back(point);
//    }
}

inline static std::string to_string(int i) {
    std::stringstream ss;
    ss << i;
    return ss.str();
}

inline static void printVector(std::vector<cv::Point2f> pts, std::string name) {
    std::cout << name << std::endl;
    for (int i = 0; i < pts.size(); i ++) {
        std::cout << i << ": " << pts[i] << std::endl;
    }
}

inline static void outputMatInfo(cv::Mat mat, std::string name) {
    std::cout << name << " type: " << mat.type() << std::endl;
    std::cout << name << " channels " << mat.channels() << std::endl;
    std::cout << name << " size " << mat.size << std::endl;
}

#endif //DEPTHEST_HELPERFUNCTIONS_H
