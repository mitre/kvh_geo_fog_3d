/**
 * @file kvh_geo_fog_3d_nodelet.cpp
 * This file uses the kvh_geo_fog_3d_driver within a ROS nodelet.
 * 
 * @author Trevor Bostic
 */

// ROS
#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include "nodelet/nodelet.h"

#include "kvh_geo_fog_3d_nodelet.hpp"

namespace kvh
{
    GEO_FOG_Nodelet::GEO_FOG_Nodelet()
    {}

    void GEO_FOG_Nodelet::onInit()
    {
        ROS_INFO("Nodelet created");
    }
}

PLUGINLIB_EXPORT_CLASS(kvh::GEO_FOG_Nodelet, nodelet::Nodelet);