#pragma once

// KVH
#include "an_packet_protocol.h"
#include "spatial_packets.h"
#include "kvh_geo_fog_3d_global_vars.hpp"

// CUSTOM ROS MESSAGES
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

// ROS
#include <ros/ros.h>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

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


// Bound rotations
double BoundFromNegPiToPi(const double);
double BoundFromNegPiToPi(const float);
double BoundFromZeroTo2Pi(const double);
double BoundFromZeroTo2Pi(const float);

// RPY to quaternion conversion
tf2::Quaternion quatFromRPY(double roll, double pitch, double yaw);

// Custom ROS Message
void PublishSystemState(ros::Publisher&, system_state_packet_t);
void PublishSatellites(ros::Publisher&, satellites_packet_t);
void PublishSatellitesDetailed(ros::Publisher&, detailed_satellites_packet_t);
void PublishLocalMagnetics(ros::Publisher&, local_magnetics_packet_t);
void PublishUtmPosition(ros::Publisher&, utm_position_packet_t);
void PublishEcefPosition(ros::Publisher&, ecef_position_packet_t);
void PublishNorthSeekingStatus(ros::Publisher&, north_seeking_status_packet_t);
void PublishKvhOdometerState(ros::Publisher&, odometer_state_packet_t);
void PublishRawSensors(ros::Publisher&, raw_sensors_packet_t);
void PublishRawGnss(ros::Publisher&, raw_gnss_packet_t);

// Standard ROS Messages
void PublishIMURaw(ros::Publisher&, system_state_packet_t);
void PublishIMURawFLU(ros::Publisher&, system_state_packet_t);
void PublishIMU_NED(ros::Publisher&, system_state_packet_t, euler_orientation_standard_deviation_packet_t);
void PublishIMU_ENU(ros::Publisher&, system_state_packet_t, euler_orientation_standard_deviation_packet_t);
void PublishIMU_RPY_NED(ros::Publisher&, system_state_packet_t);
void PublishIMU_RPY_NED_DEG(ros::Publisher&, system_state_packet_t);
void PublishIMU_RPY_ENU(ros::Publisher&, system_state_packet_t);
void PublishIMU_RPY_ENU_DEG(ros::Publisher&, system_state_packet_t);
void PublishNavSatFix(ros::Publisher&, system_state_packet_t);
void PublishRawNavSatFix(ros::Publisher&, system_state_packet_t, raw_gnss_packet_t);
void PublishMagField(ros::Publisher&, raw_sensors_packet_t);
void PublishOdomNED(ros::Publisher &, system_state_packet_t, utm_position_packet_t, 
    euler_orientation_standard_deviation_packet_t, body_velocity_packet_t);
void PublishOdomENU(ros::Publisher &, system_state_packet_t , utm_position_packet_t,
                    euler_orientation_standard_deviation_packet_t, body_velocity_packet_t);
void PublishOdomState(ros::Publisher&, odometer_state_packet_t, double);
void PublishOdomSpeed(ros::Publisher&, system_state_packet_t, odometer_state_packet_t, double, double, bool);
void PublishIMUSensorRaw(ros::Publisher&, raw_sensors_packet_t);
void PublishIMUSensorRawFLU(ros::Publisher&, raw_sensors_packet_t);
void PublishVelNEDTwist(ros::Publisher&, system_state_packet_t, velocity_standard_deviation_packet_t);
void PublishVelENUTwist(ros::Publisher&, system_state_packet_t, velocity_standard_deviation_packet_t);
void PublishVelBodyTwistFLU(ros::Publisher&, system_state_packet_t, body_velocity_packet_t, velocity_standard_deviation_packet_t);
void PublishVelBodyTwistFRD(ros::Publisher&, system_state_packet_t, body_velocity_packet_t, velocity_standard_deviation_packet_t);

