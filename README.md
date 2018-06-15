# depthest

Depth estimation based on odometry and image data.

## Installation

## Requirements

In order to run *depthest* as a part of your own setup you might need to take the following steps:

1. Calibrate your camera
2. Configure your odometry source
3. Configure your feature tracker

These steps are detailed in the following text.

### Camera calibration

The *depthest* package need to subscribe to some _image_ topic of `sensor_msgs/Image` type.
The topic name can be configured using parameter `image`.

Calibration file for the camera providing the `image` topic needs to be specified as well. This can be done using the parameter `calibration_file`. Sample configuration file can be found in the `calibration` folder. 

### Odometry

Another topic *depthest* needs to subscribe to should provide _odometry_ of the camera using `geometry_msgs/PoseStamped` type. The topic used for odometry can be specified using `odometry_topic` parameter.

### Feature tracking

Feature tracker is an important part of the *depthest* pipeline. It should publish messages of `opencv_apps/FlowArrayStampedAged` type on topic specified as `feature_tracking_topic`.

## Parameters

Here follows parameter summary for *depthest* module:

Required parameters:
- `image` - name of the topic with camera image
- `odometry_topic` - name of the topic with odometry messages
- `feature_tracking_topic` - name of the topic with output from the feature tracker
- `calibration_file` - camera calibration file

Optional parameters:
- `filter` - filtering of incorrectly estimated points. Options: `none`, `pose_filter`, `pose_change_filter`
- `enable_measurement`
- `kf_process_noise_cov`
- `kf_meas_noise_cov`
- `triangulation_sliding_window_size`
- `median_filter_sliding_window_size`
- `pose_variance`
- `pose_change_variance`
- `estimator`

## Sample rosbag run
