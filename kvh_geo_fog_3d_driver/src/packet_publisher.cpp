#include "unistd.h"
#include <map>
#include <cmath>

#include "packet_publisher.hpp"

// Bound rotations
double BoundFromNegPiToPi(const double _value)
{
    double num = std::fmod(_value, (2 * M_PI));
    if (num > M_PI)
    {
        num = num - (2 * M_PI);
    }
    return num;
}

double BoundFromNegPiToPi(const float _value)
{
    double num = std::fmod(_value, (2 * M_PI));
    if (num > M_PI)
    {
        num = num - (2 * M_PI);
    }
    return num;
}

double BoundFromZeroTo2Pi(const double _value)
{
    return std::fmod(_value, (2 * M_PI));
}

double BoundFromZeroTo2Pi(const float _value)
{
    return std::fmod(_value, (2 * M_PI));
}

tf2::Quaternion quatFromRPY(double roll, double pitch, double yaw)
{
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    return quat;
}

///////////////////////////////////////
// Custom ROS Message Publishers
///////////////////////////////////////
void PublishSystemState(ros::Publisher &_publisher, system_state_packet_t _packet)
{
    kvh_geo_fog_3d_msgs::KvhGeoFog3DSystemState sysStateMsg;
    sysStateMsg.header.stamp = ros::Time::now();

    sysStateMsg.system_status = _packet.system_status.r;
    sysStateMsg.filter_status = _packet.filter_status.r;

    sysStateMsg.unix_time_s = _packet.unix_time_seconds;
    sysStateMsg.unix_time_us = _packet.microseconds;

    sysStateMsg.latitude_rad = _packet.latitude;
    sysStateMsg.longitude_rad = _packet.longitude;
    sysStateMsg.height_m = _packet.height;

    sysStateMsg.absolute_velocity_north_mps = _packet.velocity[0];
    sysStateMsg.absolute_velocity_east_mps = _packet.velocity[1];
    sysStateMsg.absolute_velocity_down_mps = _packet.velocity[2];

    sysStateMsg.body_acceleration_x_mps = _packet.body_acceleration[0];
    sysStateMsg.body_acceleration_y_mps = _packet.body_acceleration[1];
    sysStateMsg.body_acceleration_z_mps = _packet.body_acceleration[2];

    sysStateMsg.g_force_g = _packet.g_force;

    sysStateMsg.roll_rad = _packet.orientation[0];
    sysStateMsg.pitch_rad = _packet.orientation[1];
    sysStateMsg.heading_rad = _packet.orientation[2];

    sysStateMsg.angular_velocity_x_rad_per_s = _packet.angular_velocity[0];
    sysStateMsg.angular_velocity_y_rad_per_s = _packet.angular_velocity[1];
    sysStateMsg.angular_velocity_z_rad_per_s = _packet.angular_velocity[2];

    sysStateMsg.latitude_stddev_m = _packet.standard_deviation[0];
    sysStateMsg.longitude_stddev_m = _packet.standard_deviation[1];
    sysStateMsg.height_stddev_m = _packet.standard_deviation[2];

    _publisher.publish(sysStateMsg);
}

void PublishSatellites(ros::Publisher &_publisher, satellites_packet_t _packet)
{
    kvh_geo_fog_3d_msgs::KvhGeoFog3DSatellites satellitesMsg;
    satellitesMsg.header.stamp = ros::Time::now();
    satellitesMsg.hdop = _packet.hdop;
    satellitesMsg.vdop = _packet.vdop;
    satellitesMsg.gps_satellites = _packet.gps_satellites;
    satellitesMsg.glonass_satellites = _packet.glonass_satellites;
    satellitesMsg.beidou_satellites = _packet.beidou_satellites;
    satellitesMsg.galileo_satellites = _packet.sbas_satellites;

    _publisher.publish(satellitesMsg);
}

void PublishSatellitesDetailed(ros::Publisher &_publisher, detailed_satellites_packet_t _packet)
{
    kvh_geo_fog_3d_msgs::KvhGeoFog3DDetailSatellites detailSatellitesMsg;

    detailSatellitesMsg.header.stamp = ros::Time::now();

    // MAXIMUM_DETAILED_SATELLITES is defined as 32 in spatial_packets.h
    // We must check if each field equals 0 as that denotes the end of the array
    for (int i = 0; i < MAXIMUM_DETAILED_SATELLITES; i++)
    {
        satellite_t satellite = _packet.satellites[i];

        // Check if all fields = 0, if so then we should end our loop
        if (satellite.satellite_system == 0 && satellite.number == 0 &&
            satellite.frequencies.r == 0 && satellite.elevation == 0 &&
            satellite.azimuth == 0 && satellite.snr == 0)
        {
            break;
        }

        detailSatellitesMsg.satellite_system.push_back(satellite.satellite_system);
        detailSatellitesMsg.satellite_number.push_back(satellite.number);
        detailSatellitesMsg.satellite_frequencies.push_back(satellite.frequencies.r);
        detailSatellitesMsg.elevation_deg.push_back(satellite.elevation);
        detailSatellitesMsg.azimuth_deg.push_back(satellite.azimuth);
        detailSatellitesMsg.snr_decibal.push_back(satellite.snr);
    }

    _publisher.publish(detailSatellitesMsg);
}

void PublishLocalMagnetics(ros::Publisher &_publisher, local_magnetics_packet_t _packet)
{
    kvh_geo_fog_3d_msgs::KvhGeoFog3DLocalMagneticField localMagFieldMsg;
    localMagFieldMsg.header.stamp = ros::Time::now();
    localMagFieldMsg.loc_mag_field_x_mG = _packet.magnetic_field[0];
    localMagFieldMsg.loc_mag_field_y_mG = _packet.magnetic_field[1];
    localMagFieldMsg.loc_mag_field_z_mG = _packet.magnetic_field[2];

    _publisher.publish(localMagFieldMsg);
}

void PublishUtmPosition(ros::Publisher &_publisher, utm_position_packet_t _packet)
{
    kvh_geo_fog_3d_msgs::KvhGeoFog3DUTMPosition utmPosMsg;
    utmPosMsg.header.stamp = ros::Time::now();
    utmPosMsg.northing_m = _packet.position[0];
    utmPosMsg.easting_m = _packet.position[1];
    utmPosMsg.height_m = _packet.position[2];
    utmPosMsg.zone_character = _packet.zone;

    _publisher.publish(utmPosMsg);
}

void PublishEcefPosition(ros::Publisher &_publisher, ecef_position_packet_t _packet)
{
    kvh_geo_fog_3d_msgs::KvhGeoFog3DECEFPos ecefPosMsg;
    ecefPosMsg.header.stamp = ros::Time::now();
    ecefPosMsg.ecef_x_m = _packet.position[0];
    ecefPosMsg.ecef_y_m = _packet.position[1];
    ecefPosMsg.ecef_z_m = _packet.position[2];

    _publisher.publish(ecefPosMsg);
}

void PublishNorthSeekingStatus(ros::Publisher &_publisher, north_seeking_status_packet_t _packet)
{
    kvh_geo_fog_3d_msgs::KvhGeoFog3DNorthSeekingInitStatus northSeekInitStatMsg;
    northSeekInitStatMsg.header.stamp = ros::Time::now();

    northSeekInitStatMsg.flags = _packet.north_seeking_status.r;
    northSeekInitStatMsg.quadrant_1_data_per = _packet.quadrant_data_collection_progress[0];
    northSeekInitStatMsg.quadrant_2_data_per = _packet.quadrant_data_collection_progress[1];
    northSeekInitStatMsg.quadrant_3_data_per = _packet.quadrant_data_collection_progress[2];
    northSeekInitStatMsg.quadrant_4_data_per = _packet.quadrant_data_collection_progress[3];

    northSeekInitStatMsg.current_rotation_angle_rad = _packet.current_rotation_angle;

    northSeekInitStatMsg.current_gyro_bias_sol_x_rad_s = _packet.current_gyroscope_bias_solution[0];
    northSeekInitStatMsg.current_gyro_bias_sol_y_rad_s = _packet.current_gyroscope_bias_solution[1];
    northSeekInitStatMsg.current_gyro_bias_sol_z_rad_s = _packet.current_gyroscope_bias_solution[2];
    northSeekInitStatMsg.current_gyro_bias_sol_error_per = _packet.current_gyroscope_bias_solution_error;

    _publisher.publish(northSeekInitStatMsg);
}

void PublishKvhOdometerState(ros::Publisher &_publisher, odometer_state_packet_t _packet)
{
    kvh_geo_fog_3d_msgs::KvhGeoFog3DOdometerState odometerStateMsg;
    odometerStateMsg.header.stamp = ros::Time::now();
    odometerStateMsg.odometer_pulse_count = _packet.pulse_count;
    odometerStateMsg.odometer_distance_m = _packet.distance;
    odometerStateMsg.odometer_speed_mps = _packet.speed;
    odometerStateMsg.odometer_slip_m = _packet.slip;
    odometerStateMsg.odometer_active = _packet.active;

    _publisher.publish(odometerStateMsg);
}

void PublishRawSensors(ros::Publisher &_publisher, raw_sensors_packet_t _packet)
{
    kvh_geo_fog_3d_msgs::KvhGeoFog3DRawSensors rawSensorMsg;

    rawSensorMsg.header.stamp = ros::Time::now();

    rawSensorMsg.accelerometer_x_mpss = _packet.accelerometers[0];
    rawSensorMsg.accelerometer_y_mpss = _packet.accelerometers[1];
    rawSensorMsg.accelerometer_z_mpss = _packet.accelerometers[2];

    rawSensorMsg.gyro_x_rps = _packet.gyroscopes[0];
    rawSensorMsg.gyro_y_rps = _packet.gyroscopes[1];
    rawSensorMsg.gyro_z_rps = _packet.gyroscopes[2];

    rawSensorMsg.magnetometer_x_mG = _packet.magnetometers[0];
    rawSensorMsg.magnetometer_y_mG = _packet.magnetometers[1];
    rawSensorMsg.magnetometer_z_mG = _packet.magnetometers[2];

    rawSensorMsg.imu_temp_c = _packet.imu_temperature;
    rawSensorMsg.pressure_pa = _packet.pressure;
    rawSensorMsg.pressure_temp_c = _packet.pressure_temperature;

    _publisher.publish(rawSensorMsg);
}

void PublishRawGnss(ros::Publisher &_publisher, raw_gnss_packet_t _packet)
{
    kvh_geo_fog_3d_msgs::KvhGeoFog3DRawGNSS rawGnssMsg;

    rawGnssMsg.header.stamp = ros::Time::now();
    rawGnssMsg.unix_time_s = _packet.unix_time_seconds;
    rawGnssMsg.unix_time_us = _packet.microseconds;

    rawGnssMsg.latitude_rad = _packet.position[0];
    rawGnssMsg.longitude_rad = _packet.position[1];
    rawGnssMsg.height_m = _packet.position[2];

    rawGnssMsg.latitude_stddev_m = _packet.position_standard_deviation[0];
    rawGnssMsg.longitude_stddev_m = _packet.position_standard_deviation[1];
    rawGnssMsg.height_stddev_m = _packet.position_standard_deviation[2];

    rawGnssMsg.vel_north_m = _packet.velocity[0];
    rawGnssMsg.vel_east_m = _packet.velocity[1];
    rawGnssMsg.vel_down_m = _packet.velocity[2];

    rawGnssMsg.tilt_rad = _packet.tilt;
    rawGnssMsg.tilt_stddev_rad = _packet.tilt_standard_deviation;

    rawGnssMsg.heading_rad = _packet.heading;
    rawGnssMsg.heading_stddev_rad = _packet.heading_standard_deviation;

    rawGnssMsg.gnss_fix = _packet.flags.b.fix_type;
    rawGnssMsg.doppler_velocity_valid = _packet.flags.b.velocity_valid;
    rawGnssMsg.time_valid = _packet.flags.b.time_valid;
    rawGnssMsg.external_gnss = _packet.flags.b.external_gnss;
    rawGnssMsg.tilt_valid = _packet.flags.b.tilt_valid;
    rawGnssMsg.heading_valid = _packet.flags.b.heading_valid;

    _publisher.publish(rawGnssMsg);
}

void PublishIMURaw(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket)
{
    sensor_msgs::Imu imuRaw;
    imuRaw.header.stamp = ros::Time::now();
    imuRaw.header.frame_id = "imu_link_frd";

    // \todo Fill in orientation from sysState RPY? Or from quaternion packet
    // imuRaw.orientation = ???
    // Set not to use orientation
    imuRaw.orientation_covariance[0] = -1;

    imuRaw.angular_velocity.x = _sysStatePacket.angular_velocity[0];
    imuRaw.angular_velocity.y = _sysStatePacket.angular_velocity[1];
    imuRaw.angular_velocity.z = _sysStatePacket.angular_velocity[2];
    // Leave covariance at 0 since we don't have it
    // imuRaw.angular_velocity_covariance[0]
    // imuRaw.angular_velocity_covariance[4]
    // imuRaw.angular_velocity_covariance[8]

    imuRaw.linear_acceleration.x = _sysStatePacket.body_acceleration[0];
    imuRaw.linear_acceleration.y = _sysStatePacket.body_acceleration[1];
    imuRaw.linear_acceleration.z = _sysStatePacket.body_acceleration[2];
    // Leave covariance at 0 since we don't have it
    // imuDataRaw.linear_acceleration_covariance[0]
    // imuDataRaw.linear_acceleration_covariance[4]
    // imuDataRaw.linear_acceleration_covariance[8]

    _publisher.publish(imuRaw);
}

void PublishIMURawFLU(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket)
{
    sensor_msgs::Imu imuRawFLU;
    imuRawFLU.header.stamp = ros::Time::now();
    imuRawFLU.header.frame_id = "imu_link_flu";

    // \todo Fill in orientation from sysState RPY? Or from quaternion packet
    // imuRaw.orientation = ???
    // Set not to use orientation
    imuRawFLU.orientation_covariance[0] = -1;

    // ANGULAR VELOCITY
    imuRawFLU.angular_velocity.x = _sysStatePacket.angular_velocity[0];
    imuRawFLU.angular_velocity.y = -1 * _sysStatePacket.angular_velocity[1];
    imuRawFLU.angular_velocity.z = -1 * _sysStatePacket.angular_velocity[2]; // To account for east north up system
    // Covarience should not care about offsets due to FRD to FLU conversion
    // imuRawFLU.angular_velocity_covariance[0]
    // imuRawFLU.angular_velocity_covariance[4]
    // imuRawFLU.angular_velocity_covariance[8]

    // LINEAR ACCELERATION
    imuRawFLU.linear_acceleration.x = _sysStatePacket.body_acceleration[0];
    imuRawFLU.linear_acceleration.y = -1 * _sysStatePacket.body_acceleration[1];
    imuRawFLU.linear_acceleration.z = -1 * _sysStatePacket.body_acceleration[2];
    // Leave covariance at 0 since we don't have it
    // imuRawFLU.linear_acceleration_covariance[0]
    // imuRawFLU.linear_acceleration_covariance[4]
    // imuRawFLU.linear_acceleration_covariance[8]

    _publisher.publish(imuRawFLU);
}

void PublishIMU_NED(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket, euler_orientation_standard_deviation_packet_t _eulStdDevPack)
{
    sensor_msgs::Imu imuNED;
    imuNED.header.stamp = ros::Time::now();
    imuNED.header.frame_id = "imu_link_frd";

    // Orientation
    double roll = _sysStatePacket.orientation[0];
    double pitch = _sysStatePacket.orientation[1];
    double yaw = _sysStatePacket.orientation[2];
    double yawBounded = BoundFromZeroTo2Pi(yaw);
    tf2::Quaternion q = quatFromRPY(roll, pitch, yawBounded);
    imuNED.orientation.x = q.getX();
    imuNED.orientation.y = q.getY();
    imuNED.orientation.z = q.getZ();
    imuNED.orientation.w = q.getW();
    // Orientation covariance
    imuNED.orientation_covariance[0] = pow(_eulStdDevPack.standard_deviation[0], 2);
    imuNED.orientation_covariance[4] = pow(_eulStdDevPack.standard_deviation[1], 2);
    imuNED.orientation_covariance[8] = pow(_eulStdDevPack.standard_deviation[2], 2);

    // ANGULAR VELOCITY
    imuNED.angular_velocity.x = _sysStatePacket.angular_velocity[0];
    imuNED.angular_velocity.y = _sysStatePacket.angular_velocity[1];
    imuNED.angular_velocity.z = _sysStatePacket.angular_velocity[2];
    // Covarience should not care about offsets due to FRD to FLU conversion
    // imuRawFLU.angular_velocity_covariance[0]
    // imuRawFLU.angular_velocity_covariance[4]
    // imuRawFLU.angular_velocity_covariance[8]

    // LINEAR ACCELERATION
    imuNED.linear_acceleration.x = _sysStatePacket.body_acceleration[0];
    imuNED.linear_acceleration.y = _sysStatePacket.body_acceleration[1];
    imuNED.linear_acceleration.z = _sysStatePacket.body_acceleration[2];
    // Leave covariance at 0 since we don't have it
    // imuRawFLU.linear_acceleration_covariance[0]
    // imuRawFLU.linear_acceleration_covariance[4]
    // imuRawFLU.linear_acceleration_covariance[8]

    _publisher.publish(imuNED);
}

void PublishIMU_ENU(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket, euler_orientation_standard_deviation_packet_t _eulStdDevPack)
{
    sensor_msgs::Imu imuENU;
    imuENU.header.stamp = ros::Time::now();
    imuENU.header.frame_id = "imu_link_flu";

    //For NED -> ENU transformation:
    //(X -> Y, Y -> -X, Z -> -Z, Yaw = -Yaw + 90 deg, Pitch -> -Pitch, and Roll -> Roll)
    double roll = _sysStatePacket.orientation[0];
    double pitch = -1 * _sysStatePacket.orientation[1];
    double yawNED = BoundFromZeroTo2Pi(_sysStatePacket.orientation[2]);
    double yawENU = BoundFromZeroTo2Pi(-1 * yawNED + (M_PI_2));

    tf2::Quaternion q = quatFromRPY(roll, pitch, yawENU);

    imuENU.orientation.x = q.getX();
    imuENU.orientation.y = q.getY();
    imuENU.orientation.z = q.getZ();
    imuENU.orientation.w = q.getW();

    imuENU.orientation_covariance[0] = pow(_eulStdDevPack.standard_deviation[0], 2);
    imuENU.orientation_covariance[4] = pow(_eulStdDevPack.standard_deviation[1], 2);
    imuENU.orientation_covariance[8] = pow(_eulStdDevPack.standard_deviation[2], 2);

    // ANGULAR VELOCITY
    // Keep in mind that for the sensor_msgs/Imu message, accelerations are
    // w.r.t the frame_id, which in this case is imu_link_flu.
    imuENU.angular_velocity.x = _sysStatePacket.angular_velocity[0];
    imuENU.angular_velocity.y = -1 * _sysStatePacket.angular_velocity[1];
    imuENU.angular_velocity.z = -1 * _sysStatePacket.angular_velocity[2];

    // LINEAR ACCELERATION
    // Keep in mind that for the sensor_msgs/Imu message, accelerations are
    // w.r.t the frame_id, which in this case is imu_link_flu.
    imuENU.linear_acceleration.x = _sysStatePacket.body_acceleration[0];
    imuENU.linear_acceleration.y = -1 * _sysStatePacket.body_acceleration[1];
    imuENU.linear_acceleration.z = -1 * _sysStatePacket.body_acceleration[2];

    _publisher.publish(imuENU);
}

void PublishIMU_RPY_NED(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket)
{
    geometry_msgs::Vector3Stamped imuRpyNED;
    imuRpyNED.header.stamp = ros::Time::now();
    imuRpyNED.header.frame_id = "imu_link_frd";

    double roll = _sysStatePacket.orientation[0];
    double pitch = _sysStatePacket.orientation[1];
    double yaw = _sysStatePacket.orientation[2];
    double boundedYaw = BoundFromZeroTo2Pi(yaw);

    imuRpyNED.vector.x = roll;
    imuRpyNED.vector.y = pitch;
    imuRpyNED.vector.z = boundedYaw;

    _publisher.publish(imuRpyNED);
}
void PublishIMU_RPY_NED_DEG(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket)
{
    geometry_msgs::Vector3Stamped imuRpyNEDDeg;
    imuRpyNEDDeg.header.stamp = ros::Time::now();
    imuRpyNEDDeg.header.frame_id = "imu_link_frd";

    double roll = _sysStatePacket.orientation[0];
    double pitch = _sysStatePacket.orientation[1];
    double yaw = _sysStatePacket.orientation[2];
    double boundedYaw = BoundFromZeroTo2Pi(yaw);

    imuRpyNEDDeg.vector.x = ((roll * 180.0) / M_PI);
    imuRpyNEDDeg.vector.y = ((pitch * 180.0) / M_PI);
    imuRpyNEDDeg.vector.z = ((boundedYaw * 180.0) / M_PI);

    _publisher.publish(imuRpyNEDDeg);
}

void PublishIMU_RPY_ENU(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket)
{
    geometry_msgs::Vector3Stamped imuRpyENU;
    imuRpyENU.header.stamp = ros::Time::now();
    imuRpyENU.header.frame_id = "imu_link_flu";

    //For NED -> ENU transformation:
    //(X -> Y, Y -> -X, Z -> -Z, Yaw = -Yaw + 90 deg, Pitch -> -Pitch, and Roll -> Roll)
    double roll = _sysStatePacket.orientation[0];
    double pitch = -1 * _sysStatePacket.orientation[1];
    double yawNED = BoundFromZeroTo2Pi(_sysStatePacket.orientation[2]);
    double yawENU = BoundFromZeroTo2Pi(-1 * yawNED + (M_PI_2));

    imuRpyENU.vector.x = roll;
    imuRpyENU.vector.y = pitch;
    imuRpyENU.vector.z = yawENU;

    _publisher.publish(imuRpyENU);
}

void PublishIMU_RPY_ENU_DEG(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket)
{
    geometry_msgs::Vector3Stamped imuRpyENUDeg;
    imuRpyENUDeg.header.stamp = ros::Time::now();
    imuRpyENUDeg.header.frame_id = "imu_link_flu";

    //For NED -> ENU transformation:
    //(X -> Y, Y -> -X, Z -> -Z, Yaw = -Yaw + 90 deg, Pitch -> -Pitch, and Roll -> Roll)
    double rollDeg = ((_sysStatePacket.orientation[0] * 180.0) / M_PI);
    double pitchDeg = -1 * ((_sysStatePacket.orientation[1] * 180.0) / M_PI);
    double yawNED = BoundFromZeroTo2Pi(_sysStatePacket.orientation[2]);
    double yawENU = BoundFromZeroTo2Pi(-1 * yawNED + (M_PI_2));
    double yawENUDeg = ((yawENU * 180.0) / M_PI);

    imuRpyENUDeg.vector.x = rollDeg;
    imuRpyENUDeg.vector.y = pitchDeg;
    imuRpyENUDeg.vector.z = yawENUDeg;

    _publisher.publish(imuRpyENUDeg);
}

void PublishNavSatFix(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket)
{
    sensor_msgs::NavSatFix navSatFixMsg;
    navSatFixMsg.header.stamp = ros::Time::now();
    navSatFixMsg.header.frame_id = "gps";

    // Set nav sat status
    int status = _sysStatePacket.filter_status.b.gnss_fix_type;
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

    //NavSatFix specifies degrees as lat/lon, but KVH publishes in radians
    double latitude_deg = (_sysStatePacket.latitude * 180.0) / M_PI;
    double longitude_deg = (_sysStatePacket.longitude * 180.0) / M_PI;
    navSatFixMsg.latitude = latitude_deg;
    navSatFixMsg.longitude = longitude_deg;
    navSatFixMsg.altitude = _sysStatePacket.height;
    navSatFixMsg.position_covariance_type = navSatFixMsg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
    // They use ENU for mat for this matrix.
    navSatFixMsg.position_covariance[0] = pow(_sysStatePacket.standard_deviation[1], 2);
    navSatFixMsg.position_covariance[4] = pow(_sysStatePacket.standard_deviation[0], 2);
    navSatFixMsg.position_covariance[8] = pow(_sysStatePacket.standard_deviation[2], 2);

    _publisher.publish(navSatFixMsg);
}

void PublishRawNavSatFix(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket, raw_gnss_packet_t _rawGnssPacket)
{
    sensor_msgs::NavSatFix rawNavSatFixMsg;
    rawNavSatFixMsg.header.stamp = ros::Time::now();
    rawNavSatFixMsg.header.frame_id = "gps";

    //NavSatFix specifies degrees as lat/lon, but KVH publishes in radians
    double rawGnssLatitude_deg = (_rawGnssPacket.position[0] * 180.0) / M_PI;
    double rawGnssLongitude_deg = (_rawGnssPacket.position[1] * 180.0) / M_PI;
    rawNavSatFixMsg.latitude = rawGnssLatitude_deg;
    rawNavSatFixMsg.longitude = rawGnssLongitude_deg;
    rawNavSatFixMsg.altitude = _rawGnssPacket.position[2];

    int status = _sysStatePacket.filter_status.b.gnss_fix_type;
    switch (status)
    {
    case 0:
        rawNavSatFixMsg.status.status = rawNavSatFixMsg.status.STATUS_NO_FIX;
        break;
    case 1:
    case 2:
        rawNavSatFixMsg.status.status = rawNavSatFixMsg.status.STATUS_FIX;
        break;
    case 3:
        rawNavSatFixMsg.status.status = rawNavSatFixMsg.status.STATUS_SBAS_FIX;
        break;
    default:
        rawNavSatFixMsg.status.status = rawNavSatFixMsg.status.STATUS_GBAS_FIX;
    }

    rawNavSatFixMsg.position_covariance_type = rawNavSatFixMsg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
    // They use ENU for mat for this matrix. To me it makes sense that we should use
    // the longitude standard deviation for east.
    rawNavSatFixMsg.position_covariance[0] = pow(_rawGnssPacket.position_standard_deviation[1], 2);
    rawNavSatFixMsg.position_covariance[4] = pow(_rawGnssPacket.position_standard_deviation[0], 2);
    rawNavSatFixMsg.position_covariance[8] = pow(_rawGnssPacket.position_standard_deviation[2], 2);

    _publisher.publish(rawNavSatFixMsg);
}

void PublishMagField(ros::Publisher &_publisher, raw_sensors_packet_t _rawSensorPack)
{
    sensor_msgs::MagneticField magFieldMsg;

    magFieldMsg.header.stamp = ros::Time::now();
    magFieldMsg.header.frame_id = "imu_link_frd";
    magFieldMsg.magnetic_field.x = _rawSensorPack.magnetometers[0];
    magFieldMsg.magnetic_field.y = _rawSensorPack.magnetometers[1];
    magFieldMsg.magnetic_field.z = _rawSensorPack.magnetometers[2];

    _publisher.publish(magFieldMsg);
}

void PublishOdomNED(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket, utm_position_packet_t _utmPosPacket,
                    euler_orientation_standard_deviation_packet_t _eulerStdPacket, body_velocity_packet_t _bodyVelPacket)
{
    nav_msgs::Odometry odomMsgNED;
    odomMsgNED.header.stamp = ros::Time::now();
    odomMsgNED.header.frame_id = "utm_ned";     //The nav_msgs/Odometry "Pose" section should be in this frame
    odomMsgNED.child_frame_id = "imu_link_frd"; //The nav_msgs/Odometry "Twist" section should be in this frame

    // Position NED
    odomMsgNED.pose.pose.position.x = _utmPosPacket.position[0];
    odomMsgNED.pose.pose.position.y = _utmPosPacket.position[1];
    odomMsgNED.pose.pose.position.z = -1 * _utmPosPacket.position[2];
    odomMsgNED.pose.covariance[0] = _sysStatePacket.standard_deviation[0];
    odomMsgNED.pose.covariance[7] = _sysStatePacket.standard_deviation[1];
    odomMsgNED.pose.covariance[14] = _sysStatePacket.standard_deviation[2];

    // Orientation
    double roll = _sysStatePacket.orientation[0];
    double pitch = _sysStatePacket.orientation[1];
    double yaw = BoundFromZeroTo2Pi(_sysStatePacket.orientation[2]);
    tf2::Quaternion q = quatFromRPY(roll, pitch, yaw);
    odomMsgNED.pose.pose.orientation.x = q.getX();
    odomMsgNED.pose.pose.orientation.y = q.getY();
    odomMsgNED.pose.pose.orientation.z = q.getZ();
    odomMsgNED.pose.pose.orientation.w = q.getW();

    odomMsgNED.pose.covariance[21] = pow(_eulerStdPacket.standard_deviation[1], 2);
    odomMsgNED.pose.covariance[28] = pow(_eulerStdPacket.standard_deviation[0], 2);
    odomMsgNED.pose.covariance[35] = pow(_eulerStdPacket.standard_deviation[2], 2);

    // NED uses FRD rates/accels
    odomMsgNED.twist.twist.linear.x = _bodyVelPacket.velocity[0];
    odomMsgNED.twist.twist.linear.y = _bodyVelPacket.velocity[1];
    odomMsgNED.twist.twist.linear.z = _bodyVelPacket.velocity[2];
    odomMsgNED.twist.covariance[0] = 0.0001;
    odomMsgNED.twist.covariance[7] = 0.0001;
    odomMsgNED.twist.covariance[14] = 0.0001;

    odomMsgNED.twist.twist.angular.x = _sysStatePacket.angular_velocity[0];
    odomMsgNED.twist.twist.angular.y = _sysStatePacket.angular_velocity[1];
    odomMsgNED.twist.twist.angular.z = _sysStatePacket.angular_velocity[2];
    odomMsgNED.twist.covariance[21] = 0.0001;
    odomMsgNED.twist.covariance[28] = 0.0001;
    odomMsgNED.twist.covariance[35] = 0.0001;

    _publisher.publish(odomMsgNED);
}

void PublishOdomENU(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket, utm_position_packet_t _utmPosPacket,
                    euler_orientation_standard_deviation_packet_t _eulStdPacket, body_velocity_packet_t _bodyVelPacket)
{
    nav_msgs::Odometry odomMsgENU;
    odomMsgENU.header.stamp = ros::Time::now();
    odomMsgENU.header.frame_id = "utm_enu";     //The nav_msgs/Odometry "Pose" section should be in this frame
    odomMsgENU.child_frame_id = "imu_link_flu"; //The nav_msgs/Odometry "Twist" section should be in this frame

    // POSE
    // Position ENU
    odomMsgENU.pose.pose.position.x = _utmPosPacket.position[1];
    odomMsgENU.pose.pose.position.y = _utmPosPacket.position[0];
    odomMsgENU.pose.pose.position.z = _utmPosPacket.position[2];
    odomMsgENU.pose.covariance[0] = _sysStatePacket.standard_deviation[0];
    odomMsgENU.pose.covariance[7] = _sysStatePacket.standard_deviation[1];
    odomMsgENU.pose.covariance[14] = _sysStatePacket.standard_deviation[2];

    // Orientation
    //For NED -> ENU transformation:
    //(X -> Y, Y -> -X, Z -> -Z, Yaw = -Yaw + 90 deg, Pitch -> -Pitch, and Roll -> Roll)
    double roll = _sysStatePacket.orientation[0];
    double pitch = -1 * _sysStatePacket.orientation[1];
    double yawNED = BoundFromZeroTo2Pi(_sysStatePacket.orientation[2]);
    double yawENU = (-1 * yawNED + M_PI_2);
    tf2::Quaternion q = quatFromRPY(roll, pitch, yawENU);

    // Orientation ENU
    // Use orientation quaternion we created earlier
    odomMsgENU.pose.pose.orientation.x = q.getX();
    odomMsgENU.pose.pose.orientation.y = q.getY();
    odomMsgENU.pose.pose.orientation.z = q.getZ();
    odomMsgENU.pose.pose.orientation.w = q.getW();

    // Use covariance array created earlier to fill out orientation covariance
    odomMsgENU.pose.covariance[21] = pow(_eulStdPacket.standard_deviation[1], 2);
    odomMsgENU.pose.covariance[28] = pow(_eulStdPacket.standard_deviation[0], 2);
    odomMsgENU.pose.covariance[35] = pow(_eulStdPacket.standard_deviation[2], 2);

    // ENU uses FLU rates/accels
    odomMsgENU.twist.twist.linear.x = _bodyVelPacket.velocity[0];
    odomMsgENU.twist.twist.linear.y = -(_bodyVelPacket.velocity[1]);
    odomMsgENU.twist.twist.linear.z = -(_bodyVelPacket.velocity[2]);
    odomMsgENU.twist.covariance[0] = 0.0001;
    odomMsgENU.twist.covariance[7] = 0.0001;
    odomMsgENU.twist.covariance[14] = 0.0001;

    odomMsgENU.twist.twist.angular.x = _sysStatePacket.angular_velocity[0];
    odomMsgENU.twist.twist.angular.y = (-1 * _sysStatePacket.angular_velocity[1]);
    odomMsgENU.twist.twist.angular.z = (-1 * _sysStatePacket.angular_velocity[2]);
    odomMsgENU.twist.covariance[21] = 0.0001;
    odomMsgENU.twist.covariance[28] = 0.0001;
    odomMsgENU.twist.covariance[35] = 0.0001;

    _publisher.publish(odomMsgENU);
}

void PublishOdomState(ros::Publisher &_publisher, odometer_state_packet_t _odomStatePacket, double odomPulseToMeters)
{
    nav_msgs::Odometry kvhOdomStateMsg;

    kvhOdomStateMsg.header.stamp = ros::Time::now();

    //Technically this should be w.r.t the fixed frame locked to your wheel
    //with the encoder mounted. But, since I don't know what you're going to
    //call it, we'll stick with base_link.
    kvhOdomStateMsg.header.frame_id = "base_link";

    kvhOdomStateMsg.pose.pose.position.x = (_odomStatePacket.pulse_count * odomPulseToMeters);

    kvhOdomStateMsg.pose.pose.position.x = (_odomStatePacket.pulse_count * odomPulseToMeters);
    kvhOdomStateMsg.pose.pose.position.y = 0;
    kvhOdomStateMsg.pose.pose.position.z = 0;
    kvhOdomStateMsg.twist.twist.linear.x = _odomStatePacket.speed;
    kvhOdomStateMsg.twist.twist.linear.y = 0;
    kvhOdomStateMsg.twist.twist.linear.z = 0;

    _publisher.publish(kvhOdomStateMsg);
}

void PublishOdomSpeed(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket, odometer_state_packet_t _odomStatePacket,
                      double _trackWidth, double _odometerVelocityCovariance, bool _encoderOnLeft)
{
    double yawRate = _sysStatePacket.angular_velocity[2];
    //Derived from r = v/w, with r = radius of curvature, v = velocity,  and w = angular rate,
    //and given angular rate is the same at the wheel as it is at center axle.
    double trackWidth = _trackWidth;
    double vehicleVelocity;
    if (_encoderOnLeft)
    {
        vehicleVelocity = _odomStatePacket.speed + (0.5 * trackWidth * yawRate);
    } //end: if( initOptions.encoderOnLeft )
    else
    {
        vehicleVelocity = _odomStatePacket.speed - (0.5 * trackWidth * yawRate);
    } //end: else

    geometry_msgs::TwistWithCovarianceStamped kvhOdomVehSpeedMsg;

    kvhOdomVehSpeedMsg.header.stamp = ros::Time::now();
    kvhOdomVehSpeedMsg.header.frame_id = "base_link";

    kvhOdomVehSpeedMsg.twist.twist.linear.x = vehicleVelocity;
    kvhOdomVehSpeedMsg.twist.twist.linear.y = 0.0;
    kvhOdomVehSpeedMsg.twist.twist.linear.z = 0.0;
    kvhOdomVehSpeedMsg.twist.covariance[0] = _odometerVelocityCovariance;
    kvhOdomVehSpeedMsg.twist.covariance[7] = _odometerVelocityCovariance;
    kvhOdomVehSpeedMsg.twist.covariance[14] = _odometerVelocityCovariance;

    _publisher.publish(kvhOdomVehSpeedMsg);
}

void PublishIMUSensorRaw(ros::Publisher &_publisher, raw_sensors_packet_t _rawSensorPack)
{
    sensor_msgs::Imu imuRaw;
    imuRaw.header.stamp = ros::Time::now();
    imuRaw.header.frame_id = "imu_link_frd";

    // No orientation, so set cov to -1
    imuRaw.orientation_covariance[0] = -1;

    // ANGULAR VELOCITY
    imuRaw.angular_velocity.x = _rawSensorPack.gyroscopes[0];
    imuRaw.angular_velocity.y = _rawSensorPack.gyroscopes[1];
    imuRaw.angular_velocity.z = _rawSensorPack.gyroscopes[2];

    // LINEAR ACCELERATION
    imuRaw.linear_acceleration.x = _rawSensorPack.accelerometers[0];
    imuRaw.linear_acceleration.y = _rawSensorPack.accelerometers[1];
    imuRaw.linear_acceleration.z = _rawSensorPack.accelerometers[2];

    _publisher.publish(imuRaw);
}

void PublishIMUSensorRawFLU(ros::Publisher &_publisher, raw_sensors_packet_t _rawSensorPack)
{
    sensor_msgs::Imu imuRawFLU;
    imuRawFLU.header.stamp = ros::Time::now();
    imuRawFLU.header.frame_id = "imu_link_flu";

    // ANGULAR VELOCITY
    imuRawFLU.angular_velocity.x = _rawSensorPack.gyroscopes[0];
    imuRawFLU.angular_velocity.y = -1 * _rawSensorPack.gyroscopes[1];
    imuRawFLU.angular_velocity.z = -1 * _rawSensorPack.gyroscopes[2]; // To account for east north up system

    // LINEAR ACCELERATION
    imuRawFLU.linear_acceleration.x = _rawSensorPack.accelerometers[0];
    imuRawFLU.linear_acceleration.y = -1 * _rawSensorPack.accelerometers[1];
    imuRawFLU.linear_acceleration.z = -1 * _rawSensorPack.accelerometers[2];

    _publisher.publish(imuRawFLU);
}

void PublishVelNEDTwist(ros::Publisher &_publisher, system_state_packet_t _sysStatePack, velocity_standard_deviation_packet_t _velStdPack)
{
    geometry_msgs::TwistWithCovarianceStamped velNED;
    velNED.header.frame_id = "utm_ned";
    velNED.header.stamp = ros::Time::now();
    velNED.twist.twist.linear.x = _sysStatePack.velocity[0];
    velNED.twist.twist.linear.y = _sysStatePack.velocity[1];
    velNED.twist.twist.linear.z = _sysStatePack.velocity[2];

    velNED.twist.covariance[0] = pow(_velStdPack.standard_deviation[0], 2);
    velNED.twist.covariance[7] = pow(_velStdPack.standard_deviation[1], 2);
    velNED.twist.covariance[14] = pow(_velStdPack.standard_deviation[2], 2);

    _publisher.publish(velNED);
}

void PublishVelENUTwist(ros::Publisher &_publisher, system_state_packet_t _sysStatePack, velocity_standard_deviation_packet_t _velStdPack)
{
    geometry_msgs::TwistWithCovarianceStamped velENU;
    velENU.header.frame_id = "utm_enu";
    velENU.header.stamp = ros::Time::now();
    velENU.twist.twist.linear.x = _sysStatePack.velocity[1];
    velENU.twist.twist.linear.y = _sysStatePack.velocity[0];
    velENU.twist.twist.linear.z = -1 * (_sysStatePack.velocity[2]);

    velENU.twist.covariance[0] = pow(_velStdPack.standard_deviation[1], 2);
    velENU.twist.covariance[7] = pow(_velStdPack.standard_deviation[0], 2);
    velENU.twist.covariance[14] = pow(_velStdPack.standard_deviation[2], 2);

    _publisher.publish(velENU);
}

void PublishVelBodyTwistFLU(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket,
                            body_velocity_packet_t _bodyVelPack, velocity_standard_deviation_packet_t _velStdPack)
{
    geometry_msgs::TwistWithCovarianceStamped bodyVelFLU;
    bodyVelFLU.header.frame_id = "imu_link_flu";
    bodyVelFLU.header.stamp = ros::Time::now();
    bodyVelFLU.twist.twist.linear.x = _bodyVelPack.velocity[0];
    bodyVelFLU.twist.twist.linear.y = -(_bodyVelPack.velocity[1]);
    bodyVelFLU.twist.twist.linear.z = -(_bodyVelPack.velocity[2]);

    double heading = _sysStatePacket.orientation[2];
    double bodyVelStdDevX = (_velStdPack.standard_deviation[0] * cos(heading)) - (_velStdPack.standard_deviation[1] * sin(heading));
    double bodyVelStdDevY = (_velStdPack.standard_deviation[0] * sin(heading)) + (_velStdPack.standard_deviation[1] * cos(heading));
    bodyVelFLU.twist.covariance[0] = pow(bodyVelStdDevX, 2);
    bodyVelFLU.twist.covariance[7] = pow(bodyVelStdDevY, 2);
    bodyVelFLU.twist.covariance[14] = pow(_velStdPack.standard_deviation[2], 2);

    _publisher.publish(bodyVelFLU);
}

void PublishVelBodyTwistFRD(ros::Publisher &_publisher, system_state_packet_t _sysStatePacket,
                            body_velocity_packet_t _bodyVelPack, velocity_standard_deviation_packet_t _velStdPack)
{
    geometry_msgs::TwistWithCovarianceStamped bodyVelFRD;
    bodyVelFRD.header.frame_id = "imu_link_frd";
    bodyVelFRD.header.stamp = ros::Time::now();
    bodyVelFRD.twist.twist.linear.x = _bodyVelPack.velocity[0];
    bodyVelFRD.twist.twist.linear.y = _bodyVelPack.velocity[1];
    bodyVelFRD.twist.twist.linear.z = _bodyVelPack.velocity[2];

    double heading = _sysStatePacket.orientation[2];
    double bodyVelStdDevX = (_velStdPack.standard_deviation[0] * cos(heading)) - (_velStdPack.standard_deviation[1] * sin(heading));
    double bodyVelStdDevY = (_velStdPack.standard_deviation[0] * sin(heading)) + (_velStdPack.standard_deviation[1] * cos(heading));
    bodyVelFRD.twist.covariance[0] = pow(bodyVelStdDevX, 2);
    bodyVelFRD.twist.covariance[7] = pow(bodyVelStdDevY, 2);
    bodyVelFRD.twist.covariance[14] = pow(_velStdPack.standard_deviation[2], 2);

    _publisher.publish(bodyVelFRD);
}
