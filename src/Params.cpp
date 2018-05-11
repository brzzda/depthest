//
// Created by peetaa on 24.4.2018.
//

#include <ros/ros.h>
#include "Params.h"
#include "HelperFunctions.h"

Params::Params() {
    if(!ros::param::get("/depthest/enable_measurement", enableMeasurement)) {
        ROS_INFO_STREAM("did not find 'enable_measurement' param, setting to default value 'true'");
        enableMeasurement = true;
    } else {
        ROS_INFO_STREAM("param 'enable_measurement' = " << enableMeasurement);
    }
    if(!ros::param::get("/depthest/estimator", estimator)) {
        ROS_INFO_STREAM("did not find 'estimator' param, setting to default value '" << DUMMY_POINT_FILTER << "'");
        estimator = DUMMY_POINT_FILTER;
    } else {
        ROS_INFO_STREAM("param 'filter' = " + estimator);
    }
    if(!ros::param::get("/depthest/kf_process_noise_cov", kfProcessNoiseCov)) {
        ROS_INFO_STREAM("did not find 'kf_process_noise_cov' param, setting to default value '0.01'");
        kfProcessNoiseCov = 0.01;
    } else {
        ROS_INFO_STREAM("param 'kf_process_noise_cov' = " << kfProcessNoiseCov);
    }
    if(!ros::param::get("/depthest/kf_meas_noise_cov", kfMeasNoiseCov)) {
        ROS_INFO_STREAM("did not find 'kf_meas_noise_cov' param, setting to default value '1'");
        kfMeasNoiseCov = 1;
    } else {
        ROS_INFO_STREAM("param 'kf_meas_noise_cov' = " << kfMeasNoiseCov);
    }
    if(!ros::param::get("/depthest/triangulation_sliding_window_size", triangSlidingWindowSize)) {
        ROS_INFO_STREAM("did not find 'triangulation_sliding_window_size' param, setting to default value '13'");
        triangSlidingWindowSize = 13;
    } else {
        ROS_INFO_STREAM("param 'triangulation_sliding_window_size' = " << triangSlidingWindowSize);
    }
    if(!ros::param::get("/depthest/median_filter_sliding_window_size", medianFilterSlidingWindowSize)) {
        ROS_INFO_STREAM("did not find 'median_filter_sliding_window_size' param, setting to default value '50'");
        medianFilterSlidingWindowSize = 50;
    } else {
        ROS_INFO_STREAM("param 'median_filter_sliding_window_size' = " << medianFilterSlidingWindowSize);
    }
    if(!ros::param::get("/depthest/pose_variance", poseVarianceThres)) {
        ROS_INFO_STREAM("did not find 'pose_variance' param, setting to default value '0.5'");
        poseVarianceThres = 0.5;
    } else {
        ROS_INFO_STREAM("param 'pose_variance' = " << poseVarianceThres);
    }
    if(!ros::param::get("/depthest/pose_change_variance", poseChangeVarianceThresh)) {
        ROS_INFO_STREAM("did not find 'pose_change_variance' param, setting to default value '1.0'");
        poseChangeVarianceThresh = 1.0;
    } else {
        ROS_INFO_STREAM("param 'pose_change_variance' = " << poseChangeVarianceThresh);
    }
    if(!ros::param::get("/depthest/filter", filter)) {
        ROS_INFO_STREAM("did not find 'filter' param, setting to default value '" << POSE_VARIANCE << "'");
        filter = POSE_VARIANCE;
    } else {
        ROS_INFO_STREAM("param 'filter' = " << filter);
    }
    if(!ros::param::get("/depthest/calibration_file", calibrationFileName)) {
        ROS_INFO_STREAM("did not find 'calibration_file' param, setting to default value '" << "/params/ardroneCamera.yaml" << "'");
        calibrationFileName = "/params/ardroneCamera.yaml";
    } else {
        ROS_INFO_STREAM("param 'calibration_file' = " << calibrationFileName);
    }

    if(!ros::param::get("/depthest/image", imageTopic)) {
        ROS_INFO_STREAM("did not find 'image' param, setting to default value '" << "/ardrone/image_raw" << "'");
        imageTopic = "/ardrone/image_raw";
    } else {
        ROS_INFO_STREAM("param 'image' = " << imageTopic);
    }
    if(!ros::param::get("/depthest/undist_image", undistImateTopic)) {
        ROS_INFO_STREAM("did not find 'undist_image' param, setting to default value '" << "/image_undist" << "'");
        undistImateTopic = "/image_undist";
    } else {
        ROS_INFO_STREAM("param 'undist_image' = " << undistImateTopic);
    }

    if(!ros::param::get("/depthest/odometry_topic", odometryTopic)) {
        ROS_INFO_STREAM("did not find 'odometry_topic' param, setting to default value '" << "/std_pose" << "'");
        odometryTopic = "/std_pose";
    } else {
        ROS_INFO_STREAM("param 'odometry_topic' = " << odometryTopic);
    }

    if(!ros::param::get("/depthest/feature_tracking_topic", featuresTopic)) {
        ROS_INFO_STREAM("did not find 'feature_tracking_topic' param, setting to default value '" << "/tracked_features" << "'");
        featuresTopic = "/tracked_features";
    } else {
        ROS_INFO_STREAM("param 'feature_tracking_topic' = " << featuresTopic);
    }
}

Params &Params::getInstance() {
    static Params instance;
    return instance;
}

const bool &Params::getEnabledMeasurement() const {
    return enableMeasurement;
}

const std::string Params::getEstimatorName() const {
    return estimator;
}

const float Params::getKFProcessNoise() const {
    return kfProcessNoiseCov;
}

const float Params::getKFMeasnoise() const {
    return kfMeasNoiseCov;
}

const int Params::getTriangWindowSize() const {
    return triangSlidingWindowSize;
}

const int Params::getMedianFilterWindowSize() const {
    return medianFilterSlidingWindowSize;
}

const float Params::getPoseVarianceThresth() const {
    return poseVarianceThres;
}

const float Params::getPoseChangeVarianceThresth() const {
    return poseChangeVarianceThresh;
}

const std::string Params::getFilterName() const {
    return filter;
}

const std::string Params::getCalibrationFileName() const {
    return calibrationFileName;
}

const std::string &Params::getImageTopic() const {
    return imageTopic;
}

const std::string &Params::getUndistImateTopic() const {
    return undistImateTopic;
}

const std::string &Params::getOdometryTopic() const {
    return odometryTopic;
}

const std::string &Params::getFeaturesTopic() const {
    return featuresTopic;
}
