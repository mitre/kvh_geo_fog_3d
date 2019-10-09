/*********************************************************************
 * Software License Agreement (Apache 2.0)
 * 
 *  Copyright (c) 2019, The MITRE Corporation.
 *  All rights reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *********************************************************************/

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
