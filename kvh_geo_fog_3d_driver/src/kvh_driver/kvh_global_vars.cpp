#include "kvh_geo_fog_3d_global_vars.hpp"
#include <typeinfo>

namespace kvh
{
    std::set<packet_id_e> supportedPackets_ = {
    packet_id_system_state,
    packet_id_unix_time,
    packet_id_raw_sensors,
    packet_id_satellites,
    packet_id_satellites_detailed,
    packet_id_local_magnetics,
    packet_id_utm_position,
    packet_id_ecef_position,
    packet_id_north_seeking_status,
    packet_id_euler_orientation_standard_deviation,
};

std::map<packet_id_e, int> packetSize_ = {
    {packet_id_system_state, sizeof(system_state_packet_t)},
    {packet_id_unix_time, sizeof(unix_time_packet_t)},
    {packet_id_raw_sensors, sizeof(raw_sensors_packet_t)},
    {packet_id_satellites, sizeof(satellites_packet_t)},
    {packet_id_satellites_detailed, sizeof(detailed_satellites_packet_t)},
    {packet_id_local_magnetics, sizeof(local_magnetics_packet_t)},
    {packet_id_utm_position, sizeof(utm_fix)},
    {packet_id_ecef_position, sizeof(ecef_position_packet_t)},
    {packet_id_north_seeking_status, sizeof(north_seeking_status_packet_t)},
    {packet_id_euler_orientation_standard_deviation, sizeof(euler_orientation_standard_deviation_packet_t)},
};

std::map<packet_id_e, std::string> packetTypeStr_ = {
    {packet_id_system_state, typeid(system_state_packet_t).name()},
    {packet_id_unix_time, typeid(unix_time_packet_t).name()},
    {packet_id_raw_sensors, typeid(raw_sensors_packet_t).name()},
    {packet_id_satellites, typeid(satellites_packet_t).name()},
    {packet_id_satellites_detailed, typeid(detailed_satellites_packet_t).name()},
    {packet_id_local_magnetics, typeid(local_magnetics_packet_t).name()},
    {packet_id_utm_position, typeid(utm_position_packet_t).name()},
    {packet_id_ecef_position, typeid(ecef_position_packet_t).name()},
    {packet_id_north_seeking_status, typeid(north_seeking_status_packet_t).name()},
    {packet_id_euler_orientation_standard_deviation, typeid(euler_orientation_standard_deviation_packet_t).name()},
};
}