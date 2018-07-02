#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_msgs/TFMessage.h>
#include <DepthEstimation/NodeHandles/DepthEstimationNode.h>


int main(int argc, char **argv)
{
//    cv::Mat C = (cv::Mat_<double>(3,3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    ros::init(argc, argv, "Depth_Estimation_Node");


    DepthEstimationNode de_;


    return 0;
}





