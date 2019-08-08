/**
 * @file kvh_geo_fog_3d_node.cpp
 * @brief Contains code using the kvh driver and eventually the nodelet
 * @author Trevor Bostic
 *
 * @todo Switch publishers to DiagnosticPublisher, which will let us track frequencies (see http://docs.ros.org/api/diagnostic_updater/html/classdiagnostic__updater_1_1DiagnosedPublisher.html)
 */

/**
* Messages
* We want nav_msgs/Odometry, geometry_msgs/PoseWithCovarianceStamped, geometry_msgs/TwistWithCovarianceStamped,
* sensor_msgs/Imu for our node
* 
* sensor_msgs::Imu -> imu/data_raw
* nav_msgs/Odometry -> gps/utm
*/

// STD
#include "unistd.h"
#include <map>
#include <cmath>

// KVH GEO FOG
#include "kvh_geo_fog_3d_driver.hpp"
#include "spatial_packets.h"
#include "kvh_diagnostics_container.hpp"

// ROS
#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <diagnostic_updater/diagnostic_updater.h>

// Custom ROS msgs
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DSystemState.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DSatellites.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DDetailSatellites.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DLocalMagneticField.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DUTMPosition.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DECEFPos.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DNorthSeekingInitStatus.h>

// Standard ROS msgs
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"

const static double PI = 3.14159;

// Bounds on [-pi, pi)
inline double BoundFromNegPiToPi(const double& _value)
{
  double result = _value;
  while( result < -(PI) )
  {
    result += (2*PI);
  }
  while( result >= PI )
  {
    result -= (2*PI);
  }
  return result;
} //end: BoundFromNegPiToPi(double* _value)
inline double BoundFromNegPiToPi(const float& _value)
{
  double result = _value;
  while( result < -(PI) )
  {
    result += (2*PI);
  }
  while( result >= PI )
  {
    result -= (2*PI);
  }
  return result;
} //end: BoundFromNegPiToPi(const float& _value)

// Bounds on [-pi, pi)
inline double BoundFromZeroTo2Pi(const double& _value)
{
  double result = _value;
  while( result < 0 )
  {
    result += (2*PI);
  }
  while( result >= (2*PI) )
  {
    result -= (2*PI);
  }
  return result;
} //end: BoundFromZeroTo2Pi(double* _value)
inline double BoundFromZeroTo2Pi(const float& _value)
{
  double result = _value;
  while( result < 0 )
  {
    result += (2*PI);
  }
  while( result >= (2*PI) )
  {
    result -= (2*PI);
  }
  return result;
} //end: BoundFromZeroTo2Pi(const float& _value)


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
    ros::Rate rate(50); // 50hz by default, may eventually may settable parameter

    diagnostic_updater::Updater diagnostics;
    mitre::KVH::DiagnosticsContainer diagContainer;
    SetupUpdater(&diagnostics, &diagContainer);

    // Map containing publishers for each type of message we want to send out
    std::map<packet_id_e, ros::Publisher> kvhPubMap{
        {packet_id_system_state, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DSystemState>("kvh_system_state", 1)},
        {packet_id_satellites, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DSatellites>("kvh_satellites", 1)},
        {packet_id_satellites_detailed, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DDetailSatellites>("kvh_detailed_satellites", 1)},
        {packet_id_local_magnetics, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DLocalMagneticField>("kvh_local_magnetics", 1)},
        {packet_id_utm_position, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DUTMPosition>("kvh_utm_position", 1)},
        {packet_id_ecef_position, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DECEFPos>("kvh_ecef_pos", 1)},
        {packet_id_north_seeking_status, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DNorthSeekingInitStatus>("kvh_north_seeking_status", 1)}};

    // Publishers for standard ros messages
    ros::Publisher imuDataRawPub = node.advertise<sensor_msgs::Imu>("imu/data_raw_frd", 1);
    ros::Publisher imuDataRawFLUPub = node.advertise<sensor_msgs::Imu>("imu/data_raw_flu", 1);
    ros::Publisher imuDataNEDPub = node.advertise<sensor_msgs::Imu>("imu/data_ned", 1);
    ros::Publisher imuDataENUPub = node.advertise<sensor_msgs::Imu>("imu/data_enu", 1);
    ros::Publisher imuDataRpyNEDPub = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy_ned", 1);
    ros::Publisher imuDataRpyNEDDegPub = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy_ned_deg", 1);
    ros::Publisher imuDataRpyENUPub = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy_enu", 1);
    ros::Publisher imuDataRpyENUDegPub = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy_enu_deg", 1);
    ros::Publisher navSatFixPub = node.advertise<sensor_msgs::NavSatFix>("gps/fix", 1);
    ros::Publisher magFieldPub = node.advertise<sensor_msgs::MagneticField>("mag", 1);
    ros::Publisher odomPubNED = node.advertise<nav_msgs::Odometry>("gps/utm_ned", 1);
    ros::Publisher odomPubENU = node.advertise<nav_msgs::Odometry>("gps/utm_enu", 1);

    //////////////////////////
    // KVH Setup
    //////////////////////////

    // To get packets from the driver, we first create a vector
    // that holds a pair containing the packet id and the desired frequency for it to be published
    // See documentation for all id's.
    // \todo: Put all id's we support in our documentation. Full list is in KVH's
    typedef std::pair<packet_id_e, int> freqPair;

    kvh::KvhPacketRequest packetRequest{
        freqPair(packet_id_euler_orientation_standard_deviation, 50),
        freqPair(packet_id_system_state, 50),
        freqPair(packet_id_satellites, 10),
        freqPair(packet_id_satellites_detailed, 1),
        freqPair(packet_id_local_magnetics, 50),
        freqPair(packet_id_utm_position, 50),
        freqPair(packet_id_ecef_position, 50),
        freqPair(packet_id_north_seeking_status, 50)
    };

    std::string kvhPort("/dev/ttyUSB0");
    // Can pass true to this constructor to get print outs. Is currently messy but usable
    kvh::Driver kvhDriver;
    // Check if the port has been set on the ros param server
    if (node.getParam("port", kvhPort))
    {
        ROS_INFO_STREAM("Connecting to KVH on port " << kvhPort);
    }
    else
    {
        ROS_WARN("No port specified by param, defaulting to USB0!");
    }

    kvh::KvhInitOptions initOptions;
    if (node.getParam("baud", initOptions.baudRate))
    {
        ROS_INFO_STREAM("Connecting with baud rate " << initOptions.baudRate);
    }
    else
    {
        ROS_WARN("No baud specified, using baud %d.", initOptions.baudRate);
    }

    if (node.getParam("debug", initOptions.debugOn))
    {
        if(initOptions.debugOn)
        {
            ROS_INFO_STREAM("Showing debug statements.");
        }
    }    

    kvhDriver.Init(kvhPort, packetRequest, initOptions);
    
    // Declare these for reuse
    system_state_packet_t systemStatePacket;
    satellites_packet_t satellitesPacket;
    detailed_satellites_packet_t detailSatellitesPacket;
    local_magnetics_packet_t localMagPacket;
    utm_position_packet_t utmPosPacket;
    ecef_position_packet_t ecefPosPacket;
    north_seeking_status_packet_t northSeekingStatPacket;
    euler_orientation_standard_deviation_packet_t eulStdDevPack;

    while (ros::ok())
    {
        // Collect packet data
        kvhDriver.Once();

        // Create header we will use for all messages. Important to have timestamp the same
        std_msgs::Header header;
        header.stamp = ros::Time::now();

        ///////////////////////////////////////////
        // CUSTOM ROS MESSAGES
        ///////////////////////////////////////////

        // NOTICE: To check if the packet is updated we must check the first value in the map pair.
        // packetMap[packet_id].first is true if the packet is changed. We then need to retrieve the packet.
        // To retrieve packets from the map you must do cast them since they are stored as
        // void pointers. This follows the patterns
        // struct_type_t structName = *static_cast<struct_type_t*>(packetMap[packet_id].second.get())
        // Where .second is getting the second part of the pair, and .get() is retrieving the shared pointer

        // SYSTEM STATE PACKET
        if (kvhDriver.PacketIsUpdated(packet_id_system_state))
        {   
            ROS_DEBUG("System state packet has updated. Publishing...");
            // Have to cast the shared_ptr to the correct type and then dereference.
            kvhDriver.GetPacket(packet_id_system_state, systemStatePacket);

            kvh_geo_fog_3d_msgs::KvhGeoFog3DSystemState sysStateMsg;
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

        // SATELLITES PACKET
        if (kvhDriver.PacketIsUpdated(packet_id_satellites))
        {
            ROS_DEBUG("Satellites packet updated. Publishing...");
            kvhDriver.GetPacket(packet_id_satellites, satellitesPacket);
            kvh_geo_fog_3d_msgs::KvhGeoFog3DSatellites satellitesMsg;

            satellitesMsg.header = header;
            satellitesMsg.hdop = satellitesPacket.hdop;
            satellitesMsg.vdop = satellitesPacket.vdop;
            satellitesMsg.gps_satellites = satellitesPacket.gps_satellites;
            satellitesMsg.glonass_satellites = satellitesPacket.glonass_satellites;
            satellitesMsg.beidou_satellites = satellitesPacket.beidou_satellites;
            satellitesMsg.galileo_satellites = satellitesPacket.sbas_satellites;

            kvhPubMap[packet_id_satellites].publish(satellitesMsg);
        }

        // SATELLITES DETAILED
        if (kvhDriver.PacketIsUpdated(packet_id_satellites_detailed))
        {
            ROS_DEBUG("Detailed satellites packet updated. Publishing...");
            kvhDriver.GetPacket(packet_id_satellites_detailed, detailSatellitesPacket);
            kvh_geo_fog_3d_msgs::KvhGeoFog3DDetailSatellites detailSatellitesMsg;

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

        // LOCAL MAGNETICS PACKET
        if (kvhDriver.PacketIsUpdated(packet_id_local_magnetics))
        {
            ROS_DEBUG("Local magnetics packet updated. Publishing...");
            kvhDriver.GetPacket(packet_id_local_magnetics, localMagPacket);
            kvh_geo_fog_3d_msgs::KvhGeoFog3DLocalMagneticField localMagFieldMsg;

            localMagFieldMsg.header = header;
            localMagFieldMsg.loc_mag_field_x_mG = localMagPacket.magnetic_field[0];
            localMagFieldMsg.loc_mag_field_y_mG = localMagPacket.magnetic_field[1];
            localMagFieldMsg.loc_mag_field_z_mG = localMagPacket.magnetic_field[2];

            kvhPubMap[packet_id_local_magnetics].publish(localMagFieldMsg);
        }

        // UTM POSITION PACKET
        if (kvhDriver.PacketIsUpdated(packet_id_utm_position))
        {
            ROS_DEBUG("UTM Position packet updated. Publishing...");
            kvhDriver.GetPacket(packet_id_utm_position, utmPosPacket);
            kvh_geo_fog_3d_msgs::KvhGeoFog3DUTMPosition utmPosMsg;

            utmPosMsg.header = header;
            utmPosMsg.northing_m = utmPosPacket.position[0];
            utmPosMsg.easting_m = utmPosPacket.position[1];
            utmPosMsg.height_m = utmPosPacket.position[2];
            utmPosMsg.zone_character = utmPosPacket.zone;

            kvhPubMap[packet_id_utm_position].publish(utmPosMsg);
        }

        // ECEF POSITION PACKET
        if (kvhDriver.PacketIsUpdated(packet_id_ecef_position))
        {
            ROS_DEBUG("ECEF position packet updated. Publishing...");
            kvhDriver.GetPacket(packet_id_ecef_position, ecefPosPacket);
            kvh_geo_fog_3d_msgs::KvhGeoFog3DECEFPos ecefPosMsg;

            ecefPosMsg.header = header;
            ecefPosMsg.ecef_x_m = ecefPosPacket.position[0];
            ecefPosMsg.ecef_y_m = ecefPosPacket.position[1];
            ecefPosMsg.ecef_z_m = ecefPosPacket.position[2];

            kvhPubMap[packet_id_ecef_position].publish(ecefPosMsg);
        }

        // NORTH SEEKING STATUS PACKET
        if (kvhDriver.PacketIsUpdated(packet_id_north_seeking_status))
        {
            ROS_DEBUG("North seeking status packet updated. Publishing...");
            kvhDriver.GetPacket(packet_id_north_seeking_status, northSeekingStatPacket);
            kvh_geo_fog_3d_msgs::KvhGeoFog3DNorthSeekingInitStatus northSeekInitStatMsg;

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

        ////////////////////////////////////
        // STANDARD ROS MESSAGES
        ////////////////////////////////////

        /* Logic below for the types of data we get from each packet
           if we have system state packet we can publish
           {
               IMU MSG
               NAVSATFIXMSG

               additionally, if we have the utm packet we can publish
               {
                   ODOMETRY_MSG
               }
           }
           if we hav the local magnetics packet we can publish
           {
               LOCAL_MAG_MSG
           }
        */
        if (kvhDriver.PacketIsUpdated(packet_id_system_state) && kvhDriver.PacketIsUpdated(packet_id_euler_orientation_standard_deviation))
        {
            kvhDriver.GetPacket(packet_id_system_state, systemStatePacket);
            kvhDriver.GetPacket(packet_id_euler_orientation_standard_deviation, eulStdDevPack);

            // IMU Message Structure: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html
            // Header
            // Quaternion orientation
            // float64[9] orientation_covariance
            // Vector3 angular_velocity
            // float64[9] angular_velocity_covariance
            // Vector3 linear_acceleration
            // float64[9] linear_acceleration_covariance
            // \todo fill out covariance matrices for each of the below.

            // [-pi,pi) bounded yaw
            double boundedBearingPiToPi = BoundFromNegPiToPi(systemStatePacket.orientation[2]);
            double boundedBearingZero2Pi = BoundFromZeroTo2Pi(systemStatePacket.orientation[2]);
            
            // ORIENTATION
            // All orientations from this sensor are w.r.t. true north.
            if (std::isnan(eulStdDevPack.standard_deviation[0]))
            {
              eulStdDevPack.standard_deviation[0] = 0;
              ROS_INFO("NAN Found");
            }

            if (std::isnan(eulStdDevPack.standard_deviation[1]))
            {
              eulStdDevPack.standard_deviation[1] = 0;
              ROS_INFO("NAN Found");
            }

            if (std::isnan(eulStdDevPack.standard_deviation[2]))
            {
              eulStdDevPack.standard_deviation[2] = 0;
              ROS_INFO("NAN Found");
            }


            tf2::Quaternion orientQuatNED;
            orientQuatNED.setRPY(
              systemStatePacket.orientation[0],
              systemStatePacket.orientation[1],
              boundedBearingZero2Pi);
            double orientCovNED[3] = {
              pow(eulStdDevPack.standard_deviation[0], 2),
              pow(eulStdDevPack.standard_deviation[1], 2),
              pow(eulStdDevPack.standard_deviation[2], 2)};

            tf2::Quaternion orientQuatENU;
            //For NED -> ENU transformation:
            //(X -> Y, Y -> -X, Z -> -Z, Yaw = -Yaw + 90 deg, Pitch -> Roll, and Roll -> Pitch)
            double unfixedEnuBearing = (-1 * boundedBearingZero2Pi) + (PI / 2.0);
            double enuBearing = BoundFromZeroTo2Pi(unfixedEnuBearing);
            orientQuatENU.setRPY(
              systemStatePacket.orientation[1], // ENU roll = NED pitch
              systemStatePacket.orientation[0], // ENU pitch = NED roll
              enuBearing // ENU bearing = -(NED bearing) + 90 degrees
            );
            double orientCovENU[3] = {
              pow(eulStdDevPack.standard_deviation[1], 2),
              pow(eulStdDevPack.standard_deviation[0], 2),
              pow(eulStdDevPack.standard_deviation[2], 2),
            };
            
            // DATA_RAW Topic
            sensor_msgs::Imu imuDataRaw;
            imuDataRaw.header = header;
            imuDataRaw.header.frame_id = "imu_link_frd";
            
            // ANGULAR VELOCITY
            imuDataRaw.angular_velocity.x = systemStatePacket.angular_velocity[0];
            imuDataRaw.angular_velocity.y = systemStatePacket.angular_velocity[1];
            imuDataRaw.angular_velocity.z = systemStatePacket.angular_velocity[2];
            // Leave covariance at 0 since we don't have it
            // imuDataRaw.angular_velocity_covariance[0]
            // imuDataRaw.angular_velocity_covariance[4]
            // imuDataRaw.angular_velocity_covariance[8]

            // LINEAR ACCELERATION
            imuDataRaw.linear_acceleration.x = systemStatePacket.body_acceleration[0];
            imuDataRaw.linear_acceleration.y = systemStatePacket.body_acceleration[1];
            imuDataRaw.linear_acceleration.z = systemStatePacket.body_acceleration[2];
            // Leave covariance at 0 since we don't have it
            // imuDataRaw.linear_acceleration_covariance[0]
            // imuDataRaw.linear_acceleration_covariance[4]
            // imuDataRaw.linear_acceleration_covariance[8]

            imuDataRawPub.publish(imuDataRaw);

            // DATA_RAW_FLU
            sensor_msgs::Imu imuDataRawFLU;
            imuDataRawFLU.header = header;
            imuDataRawFLU.header.frame_id = "imu_link_flu";

            // ANGULAR VELOCITY
            imuDataRawFLU.angular_velocity.x = systemStatePacket.angular_velocity[0];
            imuDataRawFLU.angular_velocity.y = -1 * systemStatePacket.angular_velocity[1];
            imuDataRawFLU.angular_velocity.z = -1 * systemStatePacket.angular_velocity[2]; // To account for east north up system
            // Leave covariance at 0 since we don't have it
            // imuDataRawFLU.angular_velocity_covariance[0]
            // imuDataRawFLU.angular_velocity_covariance[4]
            // imuDataRawFLU.angular_velocity_covariance[8]

            // LINEAR ACCELERATION
            imuDataRawFLU.linear_acceleration.x = systemStatePacket.body_acceleration[0];
            imuDataRawFLU.linear_acceleration.y = -1 * systemStatePacket.body_acceleration[1];
            imuDataRawFLU.linear_acceleration.z = -1 * systemStatePacket.body_acceleration[2];
            // Leave covariance at 0 since we don't have it
            // imuDataRawFLU.linear_acceleration_covariance[0]
            // imuDataRawFLU.linear_acceleration_covariance[4]
            // imuDataRawFLU.linear_acceleration_covariance[8]

            imuDataRawFLUPub.publish(imuDataRawFLU);

            ///////////////////////////////////////////////////////////////////////////////////////////////
            // DATA_NED topic
            ///////////////////////////////////////////////////////////////////////////////////////////////
            sensor_msgs::Imu imuDataNED;
            geometry_msgs::Vector3Stamped imuDataRpyNED;
            geometry_msgs::Vector3Stamped imuDataRpyNEDDeg;
            imuDataNED.header = header;
            imuDataNED.header.frame_id = "imu_link_frd";

            imuDataNED.orientation.x = orientQuatNED.x();
            imuDataNED.orientation.y = orientQuatNED.y();
            imuDataNED.orientation.z = orientQuatNED.z();
            imuDataNED.orientation.w = orientQuatNED.w();
            imuDataNED.orientation_covariance[0] = orientCovNED[0];
            imuDataNED.orientation_covariance[4] = orientCovNED[1];
            imuDataNED.orientation_covariance[8] = orientCovNED[2];

            imuDataRpyNED.header = header;
            imuDataRpyNED.header.frame_id = "imu_link_frd";
            imuDataRpyNED.vector.x = systemStatePacket.orientation[0];
            imuDataRpyNED.vector.y = systemStatePacket.orientation[1];
            imuDataRpyNED.vector.z = boundedBearingZero2Pi;

            imuDataRpyNEDDeg.header = header;
            imuDataRpyNEDDeg.header.frame_id = "imu_link_frd";
            imuDataRpyNEDDeg.vector.x = ((imuDataRpyNED.vector.x * 180.0) / PI);
            imuDataRpyNEDDeg.vector.y = ((imuDataRpyNED.vector.y * 180.0) / PI);
            imuDataRpyNEDDeg.vector.z = ((imuDataRpyNED.vector.z * 180.0) / PI);
            
            // ANGULAR VELOCITY
            imuDataNED.angular_velocity.x = systemStatePacket.angular_velocity[0];
            imuDataNED.angular_velocity.y = systemStatePacket.angular_velocity[1];
            imuDataNED.angular_velocity.z = systemStatePacket.angular_velocity[2];

            // LINEAR ACCELERATION
            imuDataNED.linear_acceleration.x = systemStatePacket.body_acceleration[0];
            imuDataNED.linear_acceleration.y = systemStatePacket.body_acceleration[1];
            imuDataNED.linear_acceleration.z = systemStatePacket.body_acceleration[2];

            imuDataNEDPub.publish(imuDataNED);
            imuDataRpyNEDPub.publish(imuDataRpyNED);
            imuDataRpyNEDDegPub.publish(imuDataRpyNEDDeg);
            
            ///////////////////////////////////////////////////////////////////////////////////////////////
            // DATA_ENU topic
            ///////////////////////////////////////////////////////////////////////////////////////////////
            sensor_msgs::Imu imuDataENU;
            geometry_msgs::Vector3Stamped imuDataRpyENU;
            geometry_msgs::Vector3Stamped imuDataRpyENUDeg;
            imuDataENU.header = header;
            imuDataENU.header.frame_id = "imu_link_flu";
            imuDataRpyENU.header = header;
            imuDataRpyENU.header.frame_id = "imu_link_flu";
            imuDataRpyENUDeg.header = header;
            imuDataRpyENUDeg.header.frame_id = "imu_link_flu";
            
            // ORIENTATION
            //Keep in mind that these are w.r.t. frame_id
            imuDataENU.orientation.x = orientQuatENU.x();
            imuDataENU.orientation.y = orientQuatENU.y();
            imuDataENU.orientation.z = orientQuatENU.z();
            imuDataENU.orientation.w = orientQuatENU.w();
            imuDataENU.orientation_covariance[0] = orientCovENU[1];
            imuDataENU.orientation_covariance[4] = orientCovENU[0];
            imuDataENU.orientation_covariance[8] = orientCovENU[2];

            imuDataRpyENU.vector.x = systemStatePacket.orientation[1];
            imuDataRpyENU.vector.y = systemStatePacket.orientation[0];
            imuDataRpyENU.vector.z = enuBearing;
            imuDataRpyENUDeg.vector.x = ((imuDataRpyENU.vector.x * 180.0) / PI);
            imuDataRpyENUDeg.vector.y = ((imuDataRpyENU.vector.y * 180.0) / PI);
            imuDataRpyENUDeg.vector.z = ((imuDataRpyENU.vector.z * 180.0) / PI);
            
            
            // ANGULAR VELOCITY
            // Keep in mind that for the sensor_msgs/Imu message, accelerations are
            // w.r.t the frame_id, which in this case is imu_link_flu.
            imuDataENU.angular_velocity.x = systemStatePacket.angular_velocity[0];
            imuDataENU.angular_velocity.y = -1 * systemStatePacket.angular_velocity[1];
            imuDataENU.angular_velocity.z = -1 * systemStatePacket.angular_velocity[2];

            // LINEAR ACCELERATION
            // Keep in mind that for the sensor_msgs/Imu message, accelerations are
            // w.r.t the frame_id, which in this case is imu_link_flu.
            imuDataENU.linear_acceleration.x = systemStatePacket.body_acceleration[0];
            imuDataENU.linear_acceleration.y = -1 * systemStatePacket.body_acceleration[1];
            imuDataENU.linear_acceleration.z = -1 * systemStatePacket.body_acceleration[2];

            // Publish
            imuDataENUPub.publish(imuDataENU);
            imuDataRpyENUPub.publish(imuDataRpyENU);
            imuDataRpyENUDegPub.publish(imuDataRpyENUDeg);

            // NAVSATFIX Message Structure: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html
            // NavSatStatus status
            // float64 latitude
            // float64 longitude
            // float64 altitude
            // float64[9] position_covariance // Uses East North Up (ENU) in row major order
            // uint8 position_covariance type
            sensor_msgs::NavSatFix navSatFixMsg;
            navSatFixMsg.header = header;
            navSatFixMsg.header.frame_id = "gps";

            // Set nav sat status
            int status = systemStatePacket.filter_status.b.gnss_fix_type;
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

            navSatFixMsg.latitude = systemStatePacket.latitude;
            navSatFixMsg.longitude = systemStatePacket.longitude;
            navSatFixMsg.altitude = systemStatePacket.height;
            navSatFixMsg.position_covariance_type = navSatFixMsg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
            // They use ENU for mat for this matrix. To me it makes sense that we should use
            // the longitude standard deviation for east.
            navSatFixMsg.position_covariance[0] = pow(systemStatePacket.standard_deviation[1], 2);
            navSatFixMsg.position_covariance[4] = pow(systemStatePacket.standard_deviation[0], 2);
            navSatFixMsg.position_covariance[8] = pow(systemStatePacket.standard_deviation[2], 2);

            navSatFixPub.publish(navSatFixMsg);

            // If we have system state and utm position we can publish odometry
            if (kvhDriver.PacketIsUpdated(packet_id_utm_position))
            {
                // Odometry Message Structure: http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
                // Header
                // String child_frame_id \todo Fill out child frame id
                // PoseWithCovariance pose
                // --> Pose
                // -->-->Point position
                // -->-->Quaternion orientation
                // -->float6[36] covariance // 6x6 order is [x, y, z, X axis rot, y axis rot, z axis rot]
                // TwistWithCovariance twist
                // --> Twist // Velocity in free space
                // -->-->Vector3 linear
                // -->-->Vector3 angular
                // -->float64[36] covariance // 6x6 order is [x, y, z, x axis rot, y axis rot, z axis rot]

                // Since UTM is by default NED, we will publish a message like that and a message using
                // the ros ENU standard
                nav_msgs::Odometry odomMsgENU;
                nav_msgs::Odometry odomMsgNED;
                kvhDriver.GetPacket(packet_id_utm_position, utmPosPacket);

                odomMsgENU.header = header;
                odomMsgENU.header.frame_id = "utm_enu";     //The nav_msgs/Odometry "Pose" section should be in this frame
                odomMsgENU.child_frame_id = "imu_link_flu"; //The nav_msgs/Odometry "Twist" section should be in this frame
                
                odomMsgNED.header = header;
                odomMsgNED.header.frame_id = "utm_ned";     //The nav_msgs/Odometry "Pose" section should be in this frame
                odomMsgNED.child_frame_id = "imu_link_frd"; //The nav_msgs/Odometry "Twist" section should be in this frame
                
                // \todo Fill covarience matrices for both of these
                // Covariance matrices are 6x6 so we need to fill the diagonal at
                // 0, 7, 14, 21, 28, 35

                // POSE
                // Position ENU
                odomMsgENU.pose.pose.position.x = utmPosPacket.position[1];
                odomMsgENU.pose.pose.position.y = utmPosPacket.position[0];
                odomMsgENU.pose.pose.position.z = -1 * utmPosPacket.position[2];
                // odomMsg.pose.covariance[0] =
                // odomMsg.pose.covariance[7] =
                // odomMsg.pose.covariance[14] =

                // Position NED
                odomMsgNED.pose.pose.position.x = utmPosPacket.position[0];
                odomMsgNED.pose.pose.position.y = utmPosPacket.position[1];
                odomMsgNED.pose.pose.position.z = utmPosPacket.position[2];
                // odomMsg.pose.covariance[0] =
                // odomMsg.pose.covariance[7] =
                // odomMsg.pose.covariance[14] =

                // Orientation ENU
                // Use orientation quaternion we created earlier
                odomMsgENU.pose.pose.orientation.x = orientQuatENU.x();
                odomMsgENU.pose.pose.orientation.y = orientQuatENU.y();
                odomMsgENU.pose.pose.orientation.z = orientQuatENU.z();
                odomMsgENU.pose.pose.orientation.w = orientQuatENU.w();
                // Use covariance array created earlier to fill out orientation covariance
                odomMsgENU.pose.covariance[21] = orientCovENU[0];
                odomMsgENU.pose.covariance[28] = orientCovENU[1];
                odomMsgENU.pose.covariance[35] = orientCovENU[2];

                // Orientation NED
                odomMsgNED.pose.pose.orientation.x = orientQuatNED.x();
                odomMsgNED.pose.pose.orientation.y = orientQuatNED.y();
                odomMsgNED.pose.pose.orientation.z = orientQuatNED.z();
                odomMsgNED.pose.pose.orientation.w = orientQuatNED.w();
                // Use covariance array created earlier to fill out orientation covariance
                odomMsgNED.pose.covariance[21] = orientCovNED[0];
                odomMsgNED.pose.covariance[28] = orientCovNED[1];
                odomMsgNED.pose.covariance[35] = orientCovNED[2];

                // TWIST
                // ENU uses FLU rates/accels
                odomMsgENU.twist.twist.linear.x = systemStatePacket.velocity[0];
                odomMsgENU.twist.twist.linear.y = (-1 * systemStatePacket.velocity[1]);
                odomMsgENU.twist.twist.linear.z = (-1 * systemStatePacket.velocity[2]);
                odomMsgENU.twist.twist.angular.x = systemStatePacket.angular_velocity[0];
                odomMsgENU.twist.twist.angular.y = (-1 * systemStatePacket.angular_velocity[1]);
                odomMsgENU.twist.twist.angular.z = (-1 * systemStatePacket.angular_velocity[2]);
                
                // NED uses FRD rates/accels
                odomMsgNED.twist.twist.linear.x = systemStatePacket.velocity[0];
                odomMsgNED.twist.twist.linear.y = systemStatePacket.velocity[1];
                odomMsgNED.twist.twist.linear.z = systemStatePacket.velocity[2];
                odomMsgNED.twist.twist.angular.x = systemStatePacket.angular_velocity[0];
                odomMsgNED.twist.twist.angular.y = systemStatePacket.angular_velocity[1];
                odomMsgNED.twist.twist.angular.z = systemStatePacket.angular_velocity[2];
                
                odomPubENU.publish(odomMsgENU);
                odomPubNED.publish(odomMsgNED);
            }
        }

        if (kvhDriver.PacketIsUpdated(packet_id_local_magnetics))
        {
            sensor_msgs::MagneticField magFieldMsg;
            kvhDriver.GetPacket(packet_id_local_magnetics, localMagPacket);

            magFieldMsg.header = header;
            magFieldMsg.header.frame_id = "imu_link_frd";
            magFieldMsg.magnetic_field.x = localMagPacket.magnetic_field[0];
            magFieldMsg.magnetic_field.y = localMagPacket.magnetic_field[1];
            magFieldMsg.magnetic_field.z = localMagPacket.magnetic_field[2];

            magFieldPub.publish(magFieldMsg);
        }

        // Set the "first" of all packets to false to denote they have not been updated
        kvhDriver.SetPacketUpdated(packet_id_system_state, false);
        kvhDriver.SetPacketUpdated(packet_id_satellites, false);
        kvhDriver.SetPacketUpdated(packet_id_satellites_detailed, false);
        kvhDriver.SetPacketUpdated(packet_id_utm_position, false);
        kvhDriver.SetPacketUpdated(packet_id_ecef_position, false);
        kvhDriver.SetPacketUpdated(packet_id_north_seeking_status, false);
        kvhDriver.SetPacketUpdated(packet_id_local_magnetics, false);
        kvhDriver.SetPacketUpdated(packet_id_euler_orientation_standard_deviation, false);

        diagnostics.update();

        ros::spinOnce();
        rate.sleep();
        ROS_DEBUG("----------------------------------------");
    }

    diagnostics.broadcast(diagnostic_msgs::DiagnosticStatus::WARN, "Shutting down the KVH driver");
    kvhDriver.Cleanup();
}
