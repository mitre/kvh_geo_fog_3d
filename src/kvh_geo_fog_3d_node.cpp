/**
 * @file kvh_geo_fog_3d_node.cpp
 * @brief Contains code using the kvh driver and eventually the nodelet
 * @author Trevor Bostic
 */

#include "ros/ros.h"
#include "kvh_geo_fog_3d_driver.hpp"
#include "unistd.h"
#include "spatial_packets.h"
#include <kvh_geo_fog_3d_driver/KvhGeoFog3DSystemState.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kvh_geo_fog_3d_driver");

    ros::NodeHandle node;
    ros::Publisher pub;
    pub = node.advertise<kvh_geo_fog_3d_driver::KvhGeoFog3DSystemState>("kvhsytemstate", 1);

    // Can pass true to this constructor to get print outs. Is currently messy but usable
    kvh::Driver kvhDriver;
    kvhDriver.Init();

    // To get packets from the driver, we first create a vector of the packet id's we want
    // See documentation for all id's. TODO: Put all id's in our documentation, is in KVH's
    std::vector<packet_id_e> packetRequest
    {
        packet_id_system_state, 
        packet_id_unix_time, 
        packet_id_raw_sensors
    };
    // Create a map, this will hold all of our data and status changes (if the packets were updated)
    kvh::KvhPackageMap packetMap;
    // Send the above to this function, it will initialize our map. KvhPackageMap is a messy map of type:
    // std::map<packet_id_e, std::pair<bool, std::shared_pointer<void>>>, so best not to deal with it if possible
    kvhDriver.CreatePacketMap(packetMap, packetRequest);

    system_state_packet_t systemStatePacket;

    while (ros::ok())
    {
        kvhDriver.Once(packetMap);

        // If the sytem state packet has been updated
        if (packetMap[packet_id_system_state].first)
        {
            ROS_INFO("System state packet has updated. Publishing...");
            // Have to cast the shared_ptr to the correct type and then dereference. 
            // TODO: Looking for ways to simplify this statement for driver users.
            // Static cast
            systemStatePacket = *static_cast<system_state_packet_t*>(packetMap[packet_id_system_state].second.get());

            kvh_geo_fog_3d_driver::KvhGeoFog3DSystemState sysStateMsg;
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            sysStateMsg.header = header;
            sysStateMsg.system_status = systemStatePacket.system_status.r;
            sysStateMsg.filter_status = systemStatePacket.filter_status.r;
            sysStateMsg.unix_time_s = systemStatePacket.unix_time_seconds;
            sysStateMsg.unix_time_us = systemStatePacket.microseconds;
            sysStateMsg.latitude_rad = systemStatePacket.latitude;
            sysStateMsg.longitude_rad = systemStatePacket.longitude;
            sysStateMsg.height_m = systemStatePacket.height;
            sysStateMsg.absolute_velocity_north_mps = systemStatePacket.velocity[0];
            sysStateMsg.absolute_velocity_east_mps = systemStatePacket.velocity[1];
            sysStateMsg.absolute_velocity_down_mps = systemStatePacket.velocity[2];
            sysStateMsg.body_acceleration_x_mps = systemStatePacket.body_acceleration[0];
            sysStateMsg.body_acceleration_y_mps = systemStatePacket.body_acceleration[1];
            sysStateMsg.body_acceleration_z_mps = systemStatePacket.body_acceleration[2];
            sysStateMsg.g_force_g = systemStatePacket.g_force;
            sysStateMsg.roll_rad = systemStatePacket.orientation[0];
            sysStateMsg.pitch_rad = systemStatePacket.orientation[1];
            sysStateMsg.heading_rad = systemStatePacket.orientation[2];
            sysStateMsg.angular_velocity_x_rad_per_s = systemStatePacket.angular_velocity[0];
            sysStateMsg.angular_velocity_y_rad_per_s = systemStatePacket.angular_velocity[1];
            sysStateMsg.angular_velocity_z_rad_per_s = systemStatePacket.angular_velocity[2];
            sysStateMsg.latitude_stddev_m = systemStatePacket.standard_deviation[0];
            sysStateMsg.longitude_stddev_m = systemStatePacket.standard_deviation[1];
            sysStateMsg.height_stddev_m = systemStatePacket.standard_deviation[2];

            pub.publish(sysStateMsg);
        }

        if (packetMap[packet_id_unix_time].first)
        {
            ROS_INFO("Unix packet has updated.");
        }

        if (packetMap[packet_id_raw_sensors].first)
        {
            ROS_INFO("Raw sensors packet has updated.");
        }

        usleep(100000);
    }

    kvhDriver.Cleanup();
}