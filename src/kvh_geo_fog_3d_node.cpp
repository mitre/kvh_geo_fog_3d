/**
 * @file kvh_geo_fog_3d_node.cpp
 * @brief Contains code using the kvh driver and eventually the nodelet
 * @author Trevor Bostic
 */

#include "ros/ros.h"
#include "kvh_geo_fog_3d_driver.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kvh_geo_fog_3d_driver");

    ros::NodeHandle node;

    kvh::Driver kvhDriver;
    kvhDriver.Init();
    kvhDriver.Cleanup();

}