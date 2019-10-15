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
 * @file determine_baud.cpp
 * @brief This utility can be used to determine the required baud rate given 
 * the existing packet requests. It can also be used to determine the current
 * baud of the kvh.
 * 
 * @author Trevor Bostic
 */

// STD
#include <iostream>
#include <algorithm>

// KVH GEO FOG
#include "kvh_geo_fog_3d_driver.hpp"
#include "kvh_geo_fog_3d_device_configuration.hpp"
#include "spatial_packets.h"

// ROS
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kvh_geo_fog_3d_driver");

    ros::NodeHandle node("~");

    std::set<int> baudRates = {
        1200, 1800, 2400, 4800, 9600,
        19200, 57600, 115200, 230400,
        460800, 500000, 576000, 921600, 1000000};

    // We will just look for the system state packet
    kvh::KvhPacketRequest packetRequest =
    {
        std::pair<packet_id_e, int>(packet_id_system_state, 50)
    };

    std::string kvhPort("/dev/ttyUSB0");
    // Check if the port has been set on the ros param server
    if (node.getParam("port", kvhPort))
    {
        ROS_INFO_STREAM("Connecting to KVH on port " << kvhPort);
    }
    else
    {
        ROS_WARN("No port specified by param, defaulting to USB0!");
    }


    int curBaudRate = kvh::KvhDeviceConfig::FindCurrentBaudRate(kvhPort);
    
    if (curBaudRate > 0)
    {
        int newBaudRate = 0;
        std::string possibleRates = "";

        for(int rate : baudRates)
        {
            possibleRates += std::to_string(rate) + "\n";
        }

        while (true)
        {
            printf("If you wish to modify the baud rate, please input a new rate.\n%s", possibleRates.c_str());
            printf("********************************\n");
            printf("Keep in mind that if your port runs on hyper mode, divide your baud rate\n");
            printf("by a factor of 8. Example, if you wish to enter 921600, enter 115200 (921600/115200)\n");
            printf("********************************\n");
            printf("If you wish to exit, please enter a negative number.\n");


            std::cin >> newBaudRate;

            if (baudRates.count(newBaudRate) > 0)
            {
                if (kvh::KvhDeviceConfig::SetBaudRate(kvhPort, curBaudRate, newBaudRate) != 0)
                {
                    printf("Unable to set baud rate, please try again or exit.\n");
                    continue;
                }
                else
                {
                    printf("Baud Rate successfully set. Exiting.\n");
                    break;
                }
                
            }
            else if (newBaudRate > 0)
            {
                printf("Please enter a value from the list provided.\n");
            }
            else
            {
                printf("Exiting program.\n");
                break;
            }
        }
    }
    else
    {
        printf("Unable to find baud rate. Try a different port.");
    }

    printf("Reached the end successfully.\n");
}
