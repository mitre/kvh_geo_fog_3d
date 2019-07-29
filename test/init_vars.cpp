#include <gtest/gtest.h>
#include "kvh_geo_fog_3d_driver.hpp"
#include "include/init_vars.hpp"


    kvh::KvhPacketRequest KvhPackReqEnv::zeroRequest = {};

    kvh::KvhPacketRequest KvhPackReqEnv::smallRequest = {
        freqPair(packet_id_system_state, 50),
        freqPair(packet_id_raw_sensors, 50)};

    kvh::KvhPacketRequest KvhPackReqEnv::largeRequest = {
        freqPair(packet_id_euler_orientation_standard_deviation, 50),
        freqPair(packet_id_system_state, 50),
        freqPair(packet_id_satellites, 10),
        freqPair(packet_id_satellites_detailed, 1),
        freqPair(packet_id_local_magnetics, 50),
        freqPair(packet_id_utm_position, 50),
        freqPair(packet_id_ecef_position, 50),
        freqPair(packet_id_north_seeking_status, 50)};

    kvh::KvhPacketRequest KvhPackReqEnv::largeVaryingRatesRequest = {
        freqPair(packet_id_euler_orientation_standard_deviation, 100),
        freqPair(packet_id_system_state, 50),
        freqPair(packet_id_satellites, 10),
        freqPair(packet_id_satellites_detailed, 1),
        freqPair(packet_id_local_magnetics, 37),
        freqPair(packet_id_utm_position, 80),
        freqPair(packet_id_ecef_position, 46),
        freqPair(packet_id_north_seeking_status, 12)};

    kvh::KvhPacketRequest KvhPackReqEnv::minFrequencyRequest = {
        freqPair(packet_id_system_state, 0)};

    kvh::KvhPacketRequest KvhPackReqEnv::maxFrequencyRequest = {
        freqPair(packet_id_system_state, 1000)};

    kvh::KvhPacketRequest KvhPackReqEnv::exceedingMaxFrequencyRequest = {
        freqPair(packet_id_system_state, 10000)};

    kvh::KvhPacketRequest KvhPackReqEnv::duplicateRequest = {
        freqPair(packet_id_system_state, 50),
        freqPair(packet_id_system_state, 50)};

    kvh::KvhPacketRequest KvhPackReqEnv::unsupportedRequest = {
        freqPair(packet_id_wind, 50)};

    kvh::KvhPacketRequest KvhPackReqEnv::unsupportedDuplicateRequest = {
        freqPair(packet_id_system_state, 50),
        freqPair(packet_id_system_state, 50),
        freqPair(packet_id_wind, 50),
        freqPair(packet_id_wind, 50)};