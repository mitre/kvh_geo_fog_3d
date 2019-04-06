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
    pub = node.advertise<kvh_geo_fog_3d_driver::KvhGeoFog3DSystemState>("SystemState", 1);

    kvh::Driver kvhDriver;
    kvhDriver.Init();
    bool sysStateUpdate = false;
    std::map<packet_id_e, std::pair<bool, std::shared_ptr<void>>> packetMap;

    // Create system state pair
    sysStateUpdate = false;
    std::shared_ptr<void> sysPtr = std::make_shared<system_state_packet_t>();
    std::pair<bool, std::shared_ptr<void>> sysStatePair(sysStateUpdate, sysPtr);
    packetMap[packet_id_system_state] = sysStatePair;

    system_state_packet_t systemStatePacket;

    printf("Before loop.\n");
    while(ros::ok())
    {
        printf("Calling once.\n");
        kvhDriver.Once(packetMap);
        systemStatePacket = *(system_state_packet_t *) packetMap[packet_id_system_state].second.get();

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

        usleep(100000);
    }

    kvhDriver.Cleanup();

}