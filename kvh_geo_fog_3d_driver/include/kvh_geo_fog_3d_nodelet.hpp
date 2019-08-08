/**
 * @file kvh_geo_fog_3d_nodelet.hpp
 * @brief Header file for the nodelet
 */

#pragma once
#include "ros/ros.h"
#include "nodelet/nodelet.h"

namespace kvh
{
    class GEO_FOG_Nodelet : public nodelet::Nodelet
    {
        private:
            ros::Timer timer_;

        public:
            GEO_FOG_Nodelet();
            virtual void onInit();
    };
}