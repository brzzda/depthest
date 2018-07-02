//
// Created by Peter Zborovský on 11.11.2017.
//


#include <ros/ros.h>
#include <Remapping/ImageRemaper.h>
#include <Remapping/PoseRemaper.h>


int main( int argc, char** argv ) {
    ros::init(argc, argv, "remaper");
    ImageRemaper ir;
    PoseRemaper pr;
    ros::spin();
}
