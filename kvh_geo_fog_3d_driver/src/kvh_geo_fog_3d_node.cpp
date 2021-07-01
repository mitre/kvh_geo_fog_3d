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
 *
 * Sections of this project contains content developed by The MITRE Corporation.
 * If this code is used in a deployment or embedded within another project,
 * it is requested that you send an email to opensource@mitre.org in order to
 * let us know where this software is being used.
 *********************************************************************/

/**
 * @file kvh_geo_fog_3d_node.cpp
 * @brief Contains code using the kvh driver and eventually the nodelet
 * @author Trevor Bostic
 *
 * @todo Switch publishers to DiagnosticPublisher, which will let us track frequencies (see http://docs.ros.org/api/diagnostic_updater/html/classdiagnostic__updater_1_1DiagnosedPublisher.html)
 */

// STD
#include "unistd.h"
#include <map>
#include <cmath>

// KVH GEO FOG
#include "kvh_geo_fog_3d_driver.hpp"
#include "kvh_geo_fog_3d_global_vars.hpp"
#include "spatial_packets.h"
#include "kvh_diagnostics_container.hpp"
#include "packet_publisher.hpp"

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
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DOdometerState.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DRawGNSS.h>
#include <kvh_geo_fog_3d_msgs/KvhGeoFog3DRawSensors.h>

// Standard ROS msgs
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "sensor_msgs/MagneticField.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

// // Bounds on [-pi, pi)
// inline double BoundFromNegPiToPi(const double &_value)
// {
//   double num = std::fmod(_value, (2*M_PI));
//   if (num > M_PI)
//   {
//     num = num - (2*M_PI);
//   }
//   return num;
// } //end: BoundFromNegPiToPi(double* _value)

// inline double BoundFromNegPiToPi(const float &_value)
// {
//   double num = std::fmod(_value, (2*M_PI));
//   if (num > M_PI)
//   {
//     num = num - (2*M_PI);
//   }
//   return num;
// } //end: BoundFromNegPiToPi(const float& _value)

// // Bounds on [-pi, pi)
// inline double BoundFromZeroTo2Pi(const double &_value)
// {
//   return std::fmod(_value, (2 * M_PI));
// } //end: BoundFromZeroTo2Pi(double* _value)

// inline double BoundFromZeroTo2Pi(const float &_value)
// {
//   return std::fmod(_value, (2 * M_PI));
// } //end: BoundFromZeroTo2Pi(const float& _value)

void SetupUpdater(diagnostic_updater::Updater *_diagnostics, mitre::KVH::DiagnosticsContainer *_diagContainer)
{
  _diagnostics->setHardwareID("KVH GEO FOG 3D"); ///< @todo This should probably contain the serial number of the unit, but we only get that after a message read
  /**
   * @todo Add a diagnostics expected packet frequency for important packets and verify
   */
  _diagnostics->add("KVH System", _diagContainer, &mitre::KVH::DiagnosticsContainer::UpdateSystemStatus);
  _diagnostics->add("KVH Filters", _diagContainer, &mitre::KVH::DiagnosticsContainer::UpdateFilterStatus);
}

int GetInitOptions(ros::NodeHandle &_node, kvh::KvhInitOptions &_initOptions)
{

  // Check if the port has been set on the ros param server
  _node.getParam("port", _initOptions.port);
  _node.getParam("baud", _initOptions.baudRate);
  _node.getParam("debug", _initOptions.debugOn);

  int filterVehicleType;
  if (_node.getParam("filterVehicleType", filterVehicleType))
  {
    // node.getParam doesn't have an overload for uint8_t
    _initOptions.filterVehicleType = filterVehicleType;
  }

  _node.getParam("atmosphericAltitudeEnabled", _initOptions.atmosphericAltitudeEnabled);
  _node.getParam("velocityHeadingEnabled", _initOptions.velocityHeadingEnabled);
  _node.getParam("reversingDetectionEnabled", _initOptions.reversingDetectionEnabled);
  _node.getParam("motionAnalysisEnabled", _initOptions.motionAnalysisEnabled);
  _node.getParam("odomPulseToMeters", _initOptions.odomPulseToMeters);
  _node.getParam("trackWidth", _initOptions.trackWidth);
  _node.getParam("odometerVelocityCovariance", _initOptions.odometerVelocityCovariance);
  _node.getParam("encoderOnLeft", _initOptions.encoderOnLeft);

  ROS_INFO_STREAM("Port: " << _initOptions.port);
  ROS_INFO_STREAM("Baud: " << _initOptions.baudRate);
  ROS_INFO_STREAM("Debug: " << _initOptions.debugOn);
  ROS_INFO_STREAM("Filter Vehicle Type: " << (int)_initOptions.filterVehicleType);
  ROS_INFO_STREAM("Atmospheric Altitude Enabled: " << _initOptions.atmosphericAltitudeEnabled);
  ROS_INFO_STREAM("Velocity Heading Enabled: " << _initOptions.velocityHeadingEnabled);
  ROS_INFO_STREAM("Reversing Detection Enabled: " << _initOptions.reversingDetectionEnabled);
  ROS_INFO_STREAM("Motion Analysis Enabled: " << _initOptions.motionAnalysisEnabled);
  ROS_INFO_STREAM("Odometer Pulses to Meters: " << _initOptions.odomPulseToMeters);
  ROS_INFO_STREAM("Vehicle track width: " << _initOptions.trackWidth);
  ROS_INFO_STREAM("Odometer velocity covariance: " << _initOptions.odometerVelocityCovariance);
  ROS_INFO_STREAM("Encoder on left: " << _initOptions.encoderOnLeft);

  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kvh_geo_fog_3d_driver");

  ros::NodeHandle node("~");
  ros::Rate rate(50); // 50hz by default, may eventually make settable parameter

  diagnostic_updater::Updater diagnostics;
  mitre::KVH::DiagnosticsContainer diagContainer;
  SetupUpdater(&diagnostics, &diagContainer);

  // Custom msg publishers
  std::map<packet_id_e, ros::Publisher> kvhPubMap{
      {packet_id_system_state, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DSystemState>("kvh_system_state", 1)},
      {packet_id_satellites, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DSatellites>("kvh_satellites", 1)},
      {packet_id_satellites_detailed, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DDetailSatellites>("kvh_detailed_satellites", 1)},
      {packet_id_local_magnetics, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DLocalMagneticField>("kvh_local_magnetics", 1)},
      {packet_id_utm_position, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DUTMPosition>("kvh_utm_position", 1)},
      {packet_id_ecef_position, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DECEFPos>("kvh_ecef_pos", 1)},
      {packet_id_north_seeking_status, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DNorthSeekingInitStatus>("kvh_north_seeking_status", 1)},
      {packet_id_odometer_state, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DOdometerState>("kvh_odometer_state", 1)},
      {packet_id_raw_sensors, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DRawSensors>("kvh_raw_sensors", 1)},
      {packet_id_raw_gnss, node.advertise<kvh_geo_fog_3d_msgs::KvhGeoFog3DRawGNSS>("kvh_raw_gnss", 1)}};

  // Publishers for standard ros messages
  ros::Publisher imuRawPub = node.advertise<sensor_msgs::Imu>("imu/data_raw_frd", 1);
  ros::Publisher imuRawFLUPub = node.advertise<sensor_msgs::Imu>("imu/data_raw_flu", 1);
  ros::Publisher imuNEDPub = node.advertise<sensor_msgs::Imu>("imu/data_ned", 1);
  ros::Publisher imuENUPub = node.advertise<sensor_msgs::Imu>("imu/data_enu", 1);
  ros::Publisher imuRpyNEDPub = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy_ned", 1);
  ros::Publisher imuRpyNEDDegPub = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy_ned_deg", 1);
  ros::Publisher imuRpyENUPub = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy_enu", 1);
  ros::Publisher imuRpyENUDegPub = node.advertise<geometry_msgs::Vector3Stamped>("imu/rpy_enu_deg", 1);
  ros::Publisher navSatFixPub = node.advertise<sensor_msgs::NavSatFix>("gps/fix", 1);
  ros::Publisher rawNavSatFixPub = node.advertise<sensor_msgs::NavSatFix>("gps/raw_fix", 1);
  ros::Publisher magFieldPub = node.advertise<sensor_msgs::MagneticField>("mag", 1);
  ros::Publisher odomPubNED = node.advertise<nav_msgs::Odometry>("gps/utm_ned", 1);
  ros::Publisher odomPubENU = node.advertise<nav_msgs::Odometry>("gps/utm_enu", 1);
  ros::Publisher odomStatePub = node.advertise<nav_msgs::Odometry>("odom/wheel_encoder", 1);
  ros::Publisher odomSpeedPub = node.advertise<geometry_msgs::TwistWithCovarianceStamped>("odom/encoder_vehicle_velocity", 1);
  ros::Publisher rawSensorImuPub = node.advertise<sensor_msgs::Imu>("imu/raw_sensor_frd", 1);
  ros::Publisher rawSensorImuFluPub = node.advertise<sensor_msgs::Imu>("imu/raw_sensor_flu", 1);
  ros::Publisher velNEDTwistPub = node.advertise<geometry_msgs::TwistWithCovarianceStamped>("gps/vel_ned", 1);
  ros::Publisher velENUTwistPub = node.advertise<geometry_msgs::TwistWithCovarianceStamped>("gps/vel_enu", 1);
  ros::Publisher velBodyTwistFLUPub = node.advertise<geometry_msgs::TwistWithCovarianceStamped>("imu/vel_flu", 1);
  ros::Publisher velBodyTwistFRDPub = node.advertise<geometry_msgs::TwistWithCovarianceStamped>("imu/vel_frd", 1);

  //////////////////////////
  // KVH Setup
  //////////////////////////

  // To get packets from the driver, we first create a vector
  // that holds a pair containing the packet id and the desired frequency for it to be published
  // See documentation for all id's.
  typedef std::pair<packet_id_e, int> freqPair;

  kvh::KvhPacketRequest packetRequest{
      freqPair(packet_id_euler_orientation_standard_deviation, 50),
      freqPair(packet_id_system_state, 50),
      freqPair(packet_id_satellites, 10),
      freqPair(packet_id_satellites_detailed, 1),
      freqPair(packet_id_local_magnetics, 50),
      freqPair(packet_id_utm_position, 50),
      freqPair(packet_id_ecef_position, 50),
      freqPair(packet_id_north_seeking_status, 50),
      freqPair(packet_id_odometer_state, 50),
      freqPair(packet_id_raw_sensors, 50),
      freqPair(packet_id_raw_gnss, 50),
      freqPair(packet_id_body_velocity, 50),
      freqPair(packet_id_velocity_standard_deviation, 50),
  };

  kvh::Driver kvhDriver;
  kvh::KvhInitOptions initOptions;

  if (GetInitOptions(node, initOptions) < 0)
  {
    ROS_ERROR("Unable to get init options. Exiting.");
    exit(1);
  }

  int errorCode;
  if ((errorCode = kvhDriver.Init(initOptions.port, packetRequest, initOptions)) < 0)
  {
    ROS_ERROR("Unable to initialize driver. Error Code %d", errorCode);
    exit(1);
  };

  // Request odom configuration packet to get the pulse length
  // \todo Refactor! We have to do the below due to some poor design decisions
  // on my part. The odometer configuration packet cannot be added to the
  // packet request above since adding it to there will cause it to be added
  // to the packet periods packet, which in turn throws an error.
  kvhDriver.AddPacket(packet_id_odometer_configuration);
  kvhDriver.RequestPacket(packet_id_odometer_configuration);

  // Declare these for reuse
  system_state_packet_t systemStatePacket;
  satellites_packet_t satellitesPacket;
  detailed_satellites_packet_t detailSatellitesPacket;
  local_magnetics_packet_t localMagPacket;
  kvh::utm_fix utmPosPacket;
  ecef_position_packet_t ecefPosPacket;
  north_seeking_status_packet_t northSeekingStatPacket;
  euler_orientation_standard_deviation_packet_t eulStdDevPack;
  odometer_state_packet_t odomStatePacket;
  raw_sensors_packet_t rawSensorsPacket;
  raw_gnss_packet_t rawGnssPacket;
  body_velocity_packet_t bodyVelocityPacket;
  velocity_standard_deviation_packet_t velocityStdDevPack;
  odometer_configuration_packet_t odomConfigPacket;

  // Default value for pulse to meters in case it is not updated from kvh
  double odomPulseToMeters = initOptions.odomPulseToMeters;

  while (ros::ok())
  {
    // Collect packet data
    kvhDriver.Once();

    // If packets were updated, record the data, we will use the same instance
    // of the packets for the remainder of this loop

    if (kvhDriver.PacketIsUpdated(packet_id_system_state))
    {
      ROS_DEBUG_THROTTLE(1, "System state packet has been updated.");
      kvhDriver.GetPacket(packet_id_system_state, systemStatePacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_satellites))
    {
      ROS_DEBUG_THROTTLE(1, "Satellites packet has been updated.");
      kvhDriver.GetPacket(packet_id_satellites, satellitesPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_satellites_detailed))
    {
      ROS_DEBUG_THROTTLE(1, "Detailed satellites packet has been updated.");
      kvhDriver.GetPacket(packet_id_satellites_detailed, detailSatellitesPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_local_magnetics))
    {
      ROS_DEBUG_THROTTLE(1, "Local Mag packet has been updated.");
      kvhDriver.GetPacket(packet_id_local_magnetics, localMagPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_utm_position))
    {
      ROS_DEBUG_THROTTLE(1, "Utm position packet has been updated.");
      kvhDriver.GetPacket(packet_id_utm_position, utmPosPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_ecef_position))
    {
      ROS_DEBUG_THROTTLE(1, "Ecef packet has been updated.");
      kvhDriver.GetPacket(packet_id_ecef_position, ecefPosPacket);
    }
    if (kvhDriver.PacketIsUpdated(packet_id_north_seeking_status))
    {
      ROS_DEBUG_THROTTLE(1, "North Seeking Status packet has been updated.");
      kvhDriver.GetPacket(packet_id_north_seeking_status, northSeekingStatPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_euler_orientation_standard_deviation))
    {
      ROS_DEBUG_THROTTLE(1, "Euler Std Dev packet has been updated.");
      kvhDriver.GetPacket(packet_id_euler_orientation_standard_deviation, eulStdDevPack);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_odometer_state))
    {
      ROS_DEBUG_THROTTLE(1, "Odom State packet has been updated.");
      kvhDriver.GetPacket(packet_id_odometer_state, odomStatePacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_raw_sensors))
    {
      ROS_DEBUG_THROTTLE(1, "Raw Sensors packet has been updated.");
      kvhDriver.GetPacket(packet_id_raw_sensors, rawSensorsPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_raw_gnss))
    {
      ROS_DEBUG_THROTTLE(1, "Raw Gnss packet has been updated.");
      kvhDriver.GetPacket(packet_id_raw_gnss, rawGnssPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_body_velocity))
    {
      ROS_DEBUG_THROTTLE(1, "Body Velocity packet has been updated.");
      kvhDriver.GetPacket(packet_id_body_velocity, bodyVelocityPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_odometer_configuration))
    {
      ROS_DEBUG_THROTTLE(1, "Odometer configuration packet has been updated.");
      kvhDriver.GetPacket(packet_id_odometer_configuration, odomConfigPacket);
    }

    ///////////////////////////////////////////
    // OUTPUT ROS MESSAGES AND DIAGNOSTICS
    ///////////////////////////////////////////

    // SYSTEM STATE PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_system_state))
    {
      // Messages that are dependent ONLY on the system state packet
      PublishSystemState(kvhPubMap[packet_id_system_state], systemStatePacket);
      PublishIMURaw(imuRawPub, systemStatePacket);
      PublishIMURawFLU(imuRawFLUPub, systemStatePacket);
      PublishIMU_RPY_NED(imuRpyNEDPub, systemStatePacket);
      PublishIMU_RPY_NED_DEG(imuRpyNEDDegPub, systemStatePacket);
      PublishIMU_RPY_ENU(imuRpyENUPub, systemStatePacket);
      PublishIMU_RPY_ENU_DEG(imuRpyENUDegPub, systemStatePacket);
      PublishNavSatFix(navSatFixPub, systemStatePacket);

      //Update diagnostics container from this message
      diagContainer.SetSystemStatus(systemStatePacket.system_status.r);
      diagContainer.SetFilterStatus(systemStatePacket.filter_status.r);

      // Messages dependent on system state AND euler std dev
      if (kvhDriver.PacketIsUpdated(packet_id_euler_orientation_standard_deviation))
      {

        // CAREFUL!!! Sometimes this packet will return NANs for some reason
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

        PublishIMU_NED(imuNEDPub, systemStatePacket, eulStdDevPack);
        PublishIMU_ENU(imuENUPub, systemStatePacket, eulStdDevPack);

        // System state, eul std dev, utm, and body velocity
        if (kvhDriver.PacketIsUpdated(packet_id_utm_position) &&
            kvhDriver.PacketIsUpdated(packet_id_body_velocity))
        {
          PublishOdomNED(odomPubNED, systemStatePacket, utmPosPacket, eulStdDevPack, bodyVelocityPacket);
          PublishOdomENU(odomPubENU, systemStatePacket, utmPosPacket, eulStdDevPack, bodyVelocityPacket);
        }
      }

      // System State AND Raw Gnss
      if (kvhDriver.PacketIsUpdated(packet_id_raw_gnss))
      {
        PublishRawNavSatFix(rawNavSatFixPub, systemStatePacket, rawGnssPacket);
      }

      // System State AND Odometer State
      if (kvhDriver.PacketIsUpdated(packet_id_odometer_state))
      {
        PublishOdomSpeed(odomSpeedPub, systemStatePacket, odomStatePacket, initOptions.trackWidth,
        initOptions.odometerVelocityCovariance, initOptions.encoderOnLeft);
      }

      // System State and Velocity Std Dev
      if (kvhDriver.PacketIsUpdated(packet_id_velocity_standard_deviation))
      {
        PublishVelNEDTwist(velNEDTwistPub, systemStatePacket, velocityStdDevPack);
        PublishVelENUTwist(velENUTwistPub, systemStatePacket, velocityStdDevPack);
      }

      // System state, body velocity AND Velocity Std Dev
      if (kvhDriver.PacketIsUpdated(packet_id_body_velocity) &&
        kvhDriver.PacketIsUpdated(packet_id_velocity_standard_deviation))
        {
          PublishVelBodyTwistFLU(velBodyTwistFLUPub, systemStatePacket, bodyVelocityPacket, velocityStdDevPack);
          PublishVelBodyTwistFRD(velBodyTwistFRDPub, systemStatePacket, bodyVelocityPacket, velocityStdDevPack);
        }
    }

    // SATELLITES PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_satellites))
    {
      ROS_DEBUG_THROTTLE(3, "Satellites packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_satellites, satellitesPacket);

      PublishSatellites(kvhPubMap[packet_id_satellites], satellitesPacket);
    }

    // SATELLITES DETAILED
    if (kvhDriver.PacketIsUpdated(packet_id_satellites_detailed))
    {
      ROS_DEBUG_THROTTLE(3, "Detailed satellites packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_satellites_detailed, detailSatellitesPacket);

      PublishSatellitesDetailed(kvhPubMap[packet_id_satellites_detailed], detailSatellitesPacket);
    }

    // LOCAL MAGNETICS PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_local_magnetics))
    {
      ROS_DEBUG_THROTTLE(3, "Local magnetics packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_local_magnetics, localMagPacket);

      PublishLocalMagnetics(kvhPubMap[packet_id_local_magnetics], localMagPacket);
    }

    // UTM POSITION PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_utm_position))
    {
      ROS_DEBUG_THROTTLE(3, "UTM Position packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_utm_position, utmPosPacket);

      PublishUtmPosition(kvhPubMap[packet_id_utm_position], utmPosPacket);
    }

    // ECEF POSITION PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_ecef_position))
    {
      ROS_DEBUG_THROTTLE(3, "ECEF position packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_ecef_position, ecefPosPacket);

      PublishEcefPosition(kvhPubMap[packet_id_ecef_position], ecefPosPacket);
    }

    // NORTH SEEKING STATUS PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_north_seeking_status))
    {
      ROS_DEBUG_THROTTLE(3, "North seeking status packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_north_seeking_status, northSeekingStatPacket);

      PublishNorthSeekingStatus(kvhPubMap[packet_id_north_seeking_status], northSeekingStatPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_odometer_state))
    {
      ROS_DEBUG_THROTTLE(3, "Odometer state updated. Publishing...");
      kvhDriver.GetPacket(packet_id_odometer_state, odomStatePacket);

      PublishKvhOdometerState(kvhPubMap[packet_id_odometer_state], odomStatePacket);
      PublishOdomState(odomStatePub, odomStatePacket, odomPulseToMeters);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_raw_sensors))
    {
      ROS_DEBUG_THROTTLE(3, "Raw sensors packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_raw_sensors, rawSensorsPacket);

      PublishRawSensors(kvhPubMap[packet_id_raw_sensors], rawSensorsPacket);
      PublishMagField(magFieldPub, rawSensorsPacket);
      PublishIMUSensorRaw(rawSensorImuPub, rawSensorsPacket);
      PublishIMUSensorRawFLU(rawSensorImuFluPub, rawSensorsPacket);
    }

    /**
     * @attention The raw gnss has an additional field called floating
     * ambiguity heading. It is not implemented in their api. If we wish to
     * implement this, we would need to make an extension similar to what we
     * did for the UTM packet.
     */
    if (kvhDriver.PacketIsUpdated(packet_id_raw_gnss))
    {
      ROS_DEBUG_THROTTLE(3, "Raw GNSS packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_raw_gnss, rawGnssPacket);

      PublishRawGnss(kvhPubMap[packet_id_raw_gnss], rawGnssPacket);
    }

    // Get the kvh setting for pulse length to use below in the distance travelled calculation
    if (kvhDriver.PacketIsUpdated(packet_id_odometer_configuration))
    {
      ROS_DEBUG_THROTTLE(3, "Obtaining pulse length from odometer config.");
      kvhDriver.GetPacket(packet_id_odometer_configuration, odomConfigPacket);
      // Assume if it is not 0 then it has been calibrated
      if (odomConfigPacket.pulse_length != 0)
      {
        odomPulseToMeters = odomConfigPacket.pulse_length;
      }
    }

    // Set that we have read the latest versions of all packets. There is a small possibility we miss one packet
    // between using it above and setting it here.
    kvhDriver.SetPacketUpdated(packet_id_system_state, false);
    kvhDriver.SetPacketUpdated(packet_id_satellites, false);
    kvhDriver.SetPacketUpdated(packet_id_satellites_detailed, false);
    kvhDriver.SetPacketUpdated(packet_id_utm_position, false);
    kvhDriver.SetPacketUpdated(packet_id_ecef_position, false);
    kvhDriver.SetPacketUpdated(packet_id_north_seeking_status, false);
    kvhDriver.SetPacketUpdated(packet_id_local_magnetics, false);
    kvhDriver.SetPacketUpdated(packet_id_euler_orientation_standard_deviation, false);
    kvhDriver.SetPacketUpdated(packet_id_odometer_state, false);
    kvhDriver.SetPacketUpdated(packet_id_raw_gnss, false);
    kvhDriver.SetPacketUpdated(packet_id_raw_sensors, false);
    kvhDriver.SetPacketUpdated(packet_id_body_velocity, false);
    kvhDriver.SetPacketUpdated(packet_id_velocity_standard_deviation, false);
    kvhDriver.SetPacketUpdated(packet_id_odometer_configuration, false);

    diagnostics.update();

    ros::spinOnce();
    rate.sleep();
    ROS_DEBUG_THROTTLE(3, "----------------------------------------");
  }

  diagnostics.broadcast(diagnostic_msgs::DiagnosticStatus::WARN, "Shutting down the KVH driver");
  kvhDriver.Cleanup();
}
