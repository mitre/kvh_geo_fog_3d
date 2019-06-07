/**
 * @file kvh_geo_fog_3d_node.cpp
 * @brief Contains code using the kvh driver and eventually the nodelet
 * @author Trevor Bostic
 *
 * @todo Switch publishers to DiagnosticPublisher, which will let us track frequencies (see http://docs.ros.org/api/diagnostic_updater/html/classdiagnostic__updater_1_1DiagnosedPublisher.html)
 */

/*****
 * Frame id's for messages:
 * IMU - IMU
 * NavSat - GPS
 * Odom - GPS
 * MagField - IMU
 */

// STD
#include "unistd.h"
#include <map>
#include <cmath>

// KVH GEO FOG
#include "kvh_geo_fog_3d_driver.hpp"
#include "spatial_packets.h"
#include "kvh_diagnostics_container.hpp"
#include <diagnostic_updater/diagnostic_updater.h>

// ROS
#include "ros/ros.h"
#include <kvh_geo_fog_3d_driver/KvhGeoFog3DSystemState.h>
#include <kvh_geo_fog_3d_driver/KvhGeoFog3DSatellites.h>
#include <kvh_geo_fog_3d_driver/KvhGeoFog3DDetailSatellites.h>
#include <kvh_geo_fog_3d_driver/KvhGeoFog3DLocalMagneticField.h>
#include <kvh_geo_fog_3d_driver/KvhGeoFog3DUTMPosition.h>
#include <kvh_geo_fog_3d_driver/KvhGeoFog3DECEFPos.h>
#include <kvh_geo_fog_3d_driver/KvhGeoFog3DNorthSeekingInitStatus.h>

// Standard ROS msgs
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

void SetupUpdater(diagnostic_updater::Updater *_diagnostics, mitre::KVH::DiagnosticsContainer *_diagContainer)
{
    _diagnostics->setHardwareID("KVH GEO FOG 3D"); ///< @todo This should probably contain the serial number of the unit, but we only get that after a message read
    /**
   * @todo Add a diagnostics expected packet frequency for important packets and verify
   */
    _diagnostics->add("KVH System", _diagContainer, &mitre::KVH::DiagnosticsContainer::UpdateSystemStatus);
    _diagnostics->add("KVH Filters", _diagContainer, &mitre::KVH::DiagnosticsContainer::UpdateFilterStatus);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kvh_geo_fog_3d_driver");

    ros::NodeHandle node("~");

    diagnostic_updater::Updater diagnostics;
    mitre::KVH::DiagnosticsContainer diagContainer;
    SetupUpdater(&diagnostics, &diagContainer);

    // To get packets from the driver, we first create a vector of the packet id's we want
    // See documentation for all id's. TODO: Put all id's in our documentation, is in KVH's
    std::vector<packet_id_e> packetRequest{
        packet_id_system_state,
        packet_id_satellites,
        packet_id_satellites_detailed,
        packet_id_local_magnetics,
        packet_id_utm_position,
        packet_id_ecef_position,
        packet_id_north_seeking_status};

    // Map containing publishers for each type of message we want to send out
    std::map<packet_id_e, ros::Publisher> kvhPubMap{
        {packet_id_system_state, node.advertise<kvh_geo_fog_3d_driver::KvhGeoFog3DSystemState>("kvh_system_state", 1)},
        {packet_id_satellites, node.advertise<kvh_geo_fog_3d_driver::KvhGeoFog3DSatellites>("kvh_satellites", 1)},
        {packet_id_satellites_detailed, node.advertise<kvh_geo_fog_3d_driver::KvhGeoFog3DDetailSatellites>("kvh_detailed_satellites", 1)},
        {packet_id_local_magnetics, node.advertise<kvh_geo_fog_3d_driver::KvhGeoFog3DLocalMagneticField>("kvh_local_magnetics", 1)},
        {packet_id_utm_position, node.advertise<kvh_geo_fog_3d_driver::KvhGeoFog3DUTMPosition>("kvh_utm_position", 1)},
        {packet_id_ecef_position, node.advertise<kvh_geo_fog_3d_driver::KvhGeoFog3DECEFPos>("kvh_ecef_pos", 1)},
        {packet_id_north_seeking_status, node.advertise<kvh_geo_fog_3d_driver::KvhGeoFog3DNorthSeekingInitStatus>("kvh_north_seeking_status", 1)}};

    // Publishers for standard ros messages
    ros::Publisher imuPub = node.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
    ros::Publisher navSatFixPub = node.advertise<sensor_msgs::NavSatFix>("gps/fix", 1);
    ros::Publisher magFieldPub = node.advertise<sensor_msgs::MagneticField>("mag", 1);
    ros::Publisher odomPub = node.advertise<nav_msgs::Odometry>("gps/utm", 1);

    // Can pass true to this constructor to get print outs. Is currently messy but usable
    std::string kvhPort("/dev/ttyUSB0");
    kvh::Driver kvhDriver;
    if (node.getParam("port", kvhPort))
    {
        ROS_INFO_STREAM("Connecting to KVH on port " << kvhPort);
    }
    else
    {
        ROS_WARN("No port specified by param, defaulting to USB0!");
    }
    kvhDriver.Init(kvhPort, packetRequest);

    // Create a map, this will hold all of our data and status changes (if the packets were updated)
    kvh::KvhPackageMap packetMap;
    // Send the above to this function, it will initialize our map. KvhPackageMap is a messy map of type:
    // std::map<packet_id_e, std::pair<bool, std::shared_pointer<void>>>, so best not to deal with it if possible
    int unsupported = kvhDriver.CreatePacketMap(packetMap, packetRequest);
    if (unsupported > 0)
    {
        ROS_WARN("Warning: %d requested packets are unsupported and will not be available.", unsupported);
    }

    system_state_packet_t systemStatePacket;
    satellites_packet_t satellitesPacket;
    detailed_satellites_packet_t detailSatellitesPacket;
    local_magnetics_packet_t localMagPacket;
    utm_position_packet_t utmPosPacket;
    ecef_position_packet_t ecefPosPacket;
    north_seeking_status_packet_t northSeekingStatPacket;

    while (ros::ok())
    {
        // Collect packet data
        kvhDriver.Once(packetMap);

        // Create header we will use for all messages. Important to have timestamp the same
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        // If the sytem state packet has been updated
        if (packetMap[packet_id_system_state].first)
        {
            ROS_DEBUG("System state packet has updated. Publishing...");
            // Have to cast the shared_ptr to the correct type and then dereference.
            // TODO: Looking for ways to simplify this statement for driver users.
            // Static cast
            systemStatePacket = *static_cast<system_state_packet_t *>(packetMap[packet_id_system_state].second.get());

            kvh_geo_fog_3d_driver::KvhGeoFog3DSystemState sysStateMsg;
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

            //Update diagnostics container from this message
            diagContainer.SetSystemStatus(systemStatePacket.system_status.r);
            diagContainer.SetFilterStatus(systemStatePacket.filter_status.r);

            kvhPubMap[packet_id_system_state].publish(sysStateMsg);
        }

        if (packetMap[packet_id_satellites].first)
        {
            ROS_DEBUG("Satellites packet updated. Publishing...");
            satellitesPacket = *static_cast<satellites_packet_t *>(packetMap[packet_id_satellites].second.get());
            kvh_geo_fog_3d_driver::KvhGeoFog3DSatellites satellitesMsg;

            satellitesMsg.header = header;
            satellitesMsg.hdop = satellitesPacket.hdop;
            satellitesMsg.vdop = satellitesPacket.vdop;
            satellitesMsg.gps_satellites = satellitesPacket.gps_satellites;
            satellitesMsg.glonass_satellites = satellitesPacket.glonass_satellites;
            satellitesMsg.beidou_satellites = satellitesPacket.beidou_satellites;
            satellitesMsg.galileo_satellites = satellitesPacket.sbas_satellites;

            kvhPubMap[packet_id_satellites].publish(satellitesMsg);
        }

        if (packetMap[packet_id_satellites_detailed].first)
        {
            ROS_DEBUG("Detailed satellites packet updated. Publishing...");
            detailSatellitesPacket = *static_cast<detailed_satellites_packet_t *>(packetMap[packet_id_satellites_detailed].second.get());
            kvh_geo_fog_3d_driver::KvhGeoFog3DDetailSatellites detailSatellitesMsg;

            detailSatellitesMsg.header = header;

            // MAXIMUM_DETAILED_SATELLITES is defined as 32 in spatial_packets.h
            // We must check if each field equals 0 as that denotes the end of the array
            for (int i = 0; i < MAXIMUM_DETAILED_SATELLITES; i++)
            {
                satellite_t satellite = detailSatellitesPacket.satellites[i];

                // Check if all fields = 0, if so then we should end our loop
                if (satellite.satellite_system == 0 && satellite.number == 0 &&
                    satellite.frequencies.r == 0 && satellite.elevation == 0 &&
                    satellite.azimuth == 0 && satellite.snr == 0)
                {
                    break;
                }

                // Otherwise continue adding to our message
                detailSatellitesMsg.satellite_system.push_back(satellite.satellite_system);
                detailSatellitesMsg.satellite_number.push_back(satellite.number);
                detailSatellitesMsg.satellite_frequencies.push_back(satellite.frequencies.r);
                detailSatellitesMsg.elevation_deg.push_back(satellite.elevation);
                detailSatellitesMsg.azimuth_deg.push_back(satellite.azimuth);
                detailSatellitesMsg.snr_decibal.push_back(satellite.snr);
            }

            kvhPubMap[packet_id_satellites_detailed].publish(detailSatellitesMsg);
        }

        if (packetMap[packet_id_local_magnetics].first)
        {
            ROS_DEBUG("Local magnetics packet updated. Publishing...");
            localMagPacket = *static_cast<local_magnetics_packet_t *>(packetMap[packet_id_local_magnetics].second.get());
            kvh_geo_fog_3d_driver::KvhGeoFog3DLocalMagneticField localMagFieldMsg;

            localMagFieldMsg.header = header;
            localMagFieldMsg.loc_mag_field_x_mG = localMagPacket.magnetic_field[0];
            localMagFieldMsg.loc_mag_field_y_mG = localMagPacket.magnetic_field[1];
            localMagFieldMsg.loc_mag_field_z_mG = localMagPacket.magnetic_field[2];

            kvhPubMap[packet_id_local_magnetics].publish(localMagFieldMsg);
        }

        if (packetMap[packet_id_utm_position].first)
        {
            ROS_DEBUG("UTM Position packet updated. Publishing...");
            utmPosPacket = *static_cast<utm_position_packet_t *>(packetMap[packet_id_utm_position].second.get());
            kvh_geo_fog_3d_driver::KvhGeoFog3DUTMPosition utmPosMsg;

            utmPosMsg.header = header;
            utmPosMsg.northing_m = utmPosPacket.position[0];
            utmPosMsg.easting_m = utmPosPacket.position[1];
            utmPosMsg.height_m = utmPosPacket.position[2];
            utmPosMsg.zone_character = utmPosPacket.zone;

            kvhPubMap[packet_id_utm_position].publish(utmPosMsg);
        }

        if (packetMap[packet_id_ecef_position].first)
        {
            ROS_DEBUG("ECEF position packet updated. Publishing...");
            ecefPosPacket = *static_cast<ecef_position_packet_t *>(packetMap[packet_id_ecef_position].second.get());
            kvh_geo_fog_3d_driver::KvhGeoFog3DECEFPos ecefPosMsg;

            ecefPosMsg.header = header;
            ecefPosMsg.ecef_x_m = ecefPosPacket.position[0];
            ecefPosMsg.ecef_y_m = ecefPosPacket.position[1];
            ecefPosMsg.ecef_z_m = ecefPosPacket.position[2];

            kvhPubMap[packet_id_ecef_position].publish(ecefPosMsg);
        }

        if (packetMap[packet_id_north_seeking_status].first)
        {
            ROS_DEBUG("North seeking status packet updated. Publishing...");
            northSeekingStatPacket = *static_cast<north_seeking_status_packet_t *>(packetMap[packet_id_north_seeking_status].second.get());
            kvh_geo_fog_3d_driver::KvhGeoFog3DNorthSeekingInitStatus northSeekInitStatMsg;

            northSeekInitStatMsg.header = header;
            northSeekInitStatMsg.flags = northSeekingStatPacket.north_seeking_status.r;
            northSeekInitStatMsg.quadrant_1_data_per = northSeekingStatPacket.quadrant_data_collection_progress[0];
            northSeekInitStatMsg.quadrant_2_data_per = northSeekingStatPacket.quadrant_data_collection_progress[1];
            northSeekInitStatMsg.quadrant_3_data_per = northSeekingStatPacket.quadrant_data_collection_progress[2];
            northSeekInitStatMsg.quadrant_4_data_per = northSeekingStatPacket.quadrant_data_collection_progress[3];
            northSeekInitStatMsg.current_rotation_angle_rad = northSeekingStatPacket.current_rotation_angle;
            northSeekInitStatMsg.current_gyro_bias_sol_x_rad_s = northSeekingStatPacket.current_gyroscope_bias_solution[0];
            northSeekInitStatMsg.current_gyro_bias_sol_y_rad_s = northSeekingStatPacket.current_gyroscope_bias_solution[1];
            northSeekInitStatMsg.current_gyro_bias_sol_z_rad_s = northSeekingStatPacket.current_gyroscope_bias_solution[2];
            northSeekInitStatMsg.current_gyro_bias_sol_error_per = northSeekingStatPacket.current_gyroscope_bias_solution_error;

            kvhPubMap[packet_id_north_seeking_status].publish(northSeekInitStatMsg);
        }

        // Standard ros messages. There may be multiple ways to create the same packet so we have each method listed here
        // If we have the system state packet we have the data for the imu and navsatfix packets
        if (packetMap[packet_id_system_state].first)
        {

            sensor_msgs::Imu imuMsg;
            sensor_msgs::NavSatFix navSatFixMsg;
            system_state_packet_t sysPacket = *static_cast<system_state_packet_t *>(packetMap[packet_id_system_state].second.get());

            // IMU msg
            imuMsg.header = header;
            imuMsg.header.frame_id = "imu";
            imuMsg.orientation.x = sysPacket.orientation[0];
            imuMsg.orientation.y = sysPacket.orientation[1];
            imuMsg.orientation.z = sysPacket.orientation[2];
            imuMsg.angular_velocity.x = sysPacket.angular_velocity[0];
            imuMsg.angular_velocity.y = sysPacket.angular_velocity[1];
            imuMsg.angular_velocity.z = sysPacket.angular_velocity[2];
            imuMsg.linear_acceleration.x = sysPacket.body_acceleration[0];
            imuMsg.linear_acceleration.y = sysPacket.body_acceleration[1];
            imuMsg.linear_acceleration.z = sysPacket.body_acceleration[2];

            imuPub.publish(imuMsg);

            // NavSatFix msg
            navSatFixMsg.header = header;
            navSatFixMsg.header.frame_id = "gps";

            // Set nav sat status
            int status = sysPacket.filter_status.b.gnss_fix_type;
            switch (status)
            {
                case 0: 
                    navSatFixMsg.status.status = navSatFixMsg.status.STATUS_NO_FIX;
                    break;
                case 1:
                case 2:
                    navSatFixMsg.status.status = navSatFixMsg.status.STATUS_FIX;
                    break;
                case 3:
                    navSatFixMsg.status.status = navSatFixMsg.status.STATUS_SBAS_FIX;
                    break;
                default:
                    navSatFixMsg.status.status = navSatFixMsg.status.STATUS_GBAS_FIX;
            }

            navSatFixMsg.latitude = sysPacket.latitude;
            navSatFixMsg.longitude = sysPacket.longitude;
            navSatFixMsg.altitude = sysPacket.height;
            navSatFixMsg.position_covariance_type = navSatFixMsg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
            // They use ENU for mat for this matrix. To me it makes sense that we should use
            // the longitude standard deviation for east.
            navSatFixMsg.position_covariance[0] = pow(sysPacket.standard_deviation[1], 2);
            navSatFixMsg.position_covariance[4] = pow(sysPacket.standard_deviation[0], 2);
            navSatFixMsg.position_covariance[8] = pow(sysPacket.standard_deviation[2], 2);

            navSatFixPub.publish(navSatFixMsg);

            // If we have system state and utm position we can publish odometry
            if (packetMap[packet_id_utm_position].first)
            {
                nav_msgs::Odometry odomMsg;
                utm_position_packet_t utmPacket = *static_cast<utm_position_packet_t *>(packetMap[packet_id_utm_position].second.get());

                odomMsg.header = header;
                odomMsg.header.frame_id = "gps";
                odomMsg.pose.pose.position.x = utmPacket.position[0];
                odomMsg.pose.pose.position.y = utmPacket.position[1];
                odomMsg.pose.pose.position.z = utmPacket.position[2];
                odomMsg.pose.pose.orientation.x = sysPacket.orientation[0];
                odomMsg.pose.pose.orientation.y = sysPacket.orientation[1];
                odomMsg.pose.pose.orientation.z = sysPacket.orientation[2];
                odomMsg.twist.twist.angular.x = sysPacket.velocity[0];
                odomMsg.twist.twist.angular.y = sysPacket.velocity[1];
                odomMsg.twist.twist.angular.z = sysPacket.velocity[2];

                odomPub.publish(odomMsg);
            }
        }

        if (packetMap[packet_id_local_magnetics].first)
        {
            sensor_msgs::MagneticField magFieldMsg;
            local_magnetics_packet_t localMagPacket = *static_cast<local_magnetics_packet_t *>(packetMap[packet_id_local_magnetics].second.get());

            magFieldMsg.header = header;
            magFieldMsg.header.frame_id = "imu";
            magFieldMsg.magnetic_field.x = localMagPacket.magnetic_field[0];
            magFieldMsg.magnetic_field.y = localMagPacket.magnetic_field[1];
            magFieldMsg.magnetic_field.z = localMagPacket.magnetic_field[2];

            magFieldPub.publish(magFieldMsg);
        }

        diagnostics.update();

        usleep(100000);
        ROS_DEBUG("----------------------------------------");
    }

    diagnostics.broadcast(diagnostic_msgs::DiagnosticStatus::WARN, "Shutting down the KVH driver");
    kvhDriver.Cleanup();
}
