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
#include "spatial_packets.h"

// ROS
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kvh_geo_fog_3d_driver");

    ros::NodeHandle node;

    std::vector<int> baudRates = {
        1200, 1800, 2400, 4800, 9600,
        19200, 57600, 115200, 230400,
        460800, 500000, 576000, 921600, 1000000};

    // We will just look for the system state packet
    kvh::KvhPacketRequest packetRequest =
        {
            std::pair<packet_id_e, int>(packet_id_system_state, 50)};

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
    kvh::KvhInitOptions initOptions;
    int baudRateAttempt = 0;
    ros::Rate rate(10);
    bool baudFound = false;
    while (baudRateAttempt < baudRates.size() && !baudFound)
    {
        printf("Attempting Baud Rate: %d\n", baudRates[baudRateAttempt]);

        // Initialize the driver at some baud rate
        initOptions.baudRate = baudRates[baudRateAttempt];
        int error = kvhDriver.Init(kvhPort, packetRequest, initOptions);

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
                baudFound = true;
                break;
            }

            ros::spinOnce();
            rate.sleep();
        }

        // Check if baud was found before increment
        if (!baudFound)
            baudRateAttempt++;

        // Close the kvh so that we can reinitialize
        kvhDriver.Cleanup();
    }

    if (baudFound)
    {
        int newBaudRate;
        bool setRate = false;

        while (!setRate)
        {
            printf("If you wish to modify the baud rate,"
                   "please input a new baud rate, if not enter -1.\n");
            printf("Acceptable baud rates:\n");
            for (int i = 0; i < baudRates.size(); i++)
            {
                printf("%d\n", baudRates[i]);
            }

            std::cin >> newBaudRate;

            // Get the new baud rate, see if it is in the list of accepted baud rates.
            if (std::find(baudRates.begin(), baudRates.end(), newBaudRate) != baudRates.end())
            {
                printf("Setting new baud rate at %d\n", newBaudRate);
                int rv = kvh::Driver::SetBaudRate(kvhPort, baudRates[baudRateAttempt], newBaudRate);

                if (rv == 0)
                    printf("Success\n");
                else
                    printf("Failure\n");

                setRate = true;
            }
            else if (newBaudRate >= 0)
            {
                printf("Please enter a baudrate from the list of acceptable rates.\n");
            }
            else // Negative number
            {
                printf("Exiting\n");
                setRate = true;
            }
        }
    }

    printf("Reached the end successfully.\n");
}