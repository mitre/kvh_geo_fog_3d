/**
 * @file determine_baud.cpp
 * @brief This utility can be used to determine the required baud rate given 
 * the existing packet requests. It can also be used to determine the current
 * baud of the kvh.
 * 
 * @author Trevor Bostic
 */

// STD
#include <memory>

// KVH GEO FOG
#include "kvh_geo_fog_3d_driver.hpp"
#include "spatial_packets.h"

// ROS
#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kvh_geo_fog_3d_driver");

    ros::NodeHandle node;

    std::vector<int> baudRates = {
        1200, 1800, 2400, 4800, 9600,
        19200, 57600, 115200, 230400,
        460800, 500000, 576000, 921600, 1000000
    };

    // We will just look for the system state packet
    kvh::KvhPacketRequest packetRequest = 
    {
        std::pair<packet_id_e, int>(packet_id_system_state, 50)
    };

    // Create a map, this will hold all of our data and status changes (if the packets were updated)
    kvh::KvhPacketMap packetMap;

    // Send the above to this function, it will initialize our map. KvhPackageMap is a messy map of type:
    // std::map<packet_id_e, std::pair<bool, std::shared_pointer<void>>>, so best not to deal with it if possible
    int unsupported = kvh::Driver::CreatePacketMap(packetMap, packetRequest);

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
    
    // Determine if any of the requested packets are unsupported
    if (unsupported > 0)
    {
        ROS_WARN("Warning: %d requested packets are unsupported and will not be available.", unsupported);
    }

    kvh::Driver kvhDriver;
    int baudRateAttempt = 0;
    ros::Rate rate(10);
    int timerCount = 0;
    bool baudNotFound = true;
    while (baudRateAttempt < baudRates.size() && baudNotFound)
    {
        printf("Attempting Baud Rate: %d\n", baudRates[baudRateAttempt]);

        // Initialize the driver at some baud rate
        int error = kvhDriver.Init(kvhPort, packetRequest, baudRates[baudRateAttempt]);

        if (error != 0)
            printf("Error initializing: %d\n", error);

        
        // Give it ~2 seconds to see if any data comes in
        for (int timerCount = 0; timerCount < 20; timerCount++)
        {
            // Look for data
            kvhDriver.Once(packetMap);

            if (packetMap[packet_id_system_state].first)
            {
                // If we recieved data, then print out the baud rate
                // \todo Possibly put this on the rosparam server
                printf("Recieved packet. Correct baud rate found!\n");
                printf("Baud Rate: %d\n", baudRates[baudRateAttempt]);
                baudNotFound = false;
                break;

            }

            ros::spinOnce();
            rate.sleep();
        }

        baudRateAttempt++;
        // Close the file so that we can reinitialize
        kvhDriver.Cleanup();
    }

    printf("Reached the end successfully.\n");
}