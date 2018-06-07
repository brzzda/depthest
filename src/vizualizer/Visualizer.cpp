//
// Created by Peter Zborovský on 28.10.2017.
//


#include <ros/ros.h>
#include <vizualizer/Visualizer3D.h>
#include <vizualizer/DepthVisualizer.h>
#include <vizualizer/ReprojectionVisualizer.h>



int main( int argc, char** argv ) {
    ros::init(argc, argv, "visualizer");
    Visualizer3D pv;
    DepthVisualizer dv;
    ReprojectionVisualizer rv;
    ros::spin();
}
