#pragma once

// STD
#include <stdint.h>

// KVH Geo Fog
#include "an_packet_protocol.h"
#include "spatial_packets.h"

// Driver
#include "kvh_geo_fog_3d_global_vars.hpp"
#include "kvh_geo_fog_3d_packet_storage.hpp"

namespace kvh
{

class KvhDeviceConfig
{
private:
public:
    /**
     * @code
     * kvh::KvhPacketRequest packRequest = {
     *    // Fill in packets
     * }
     * packet_periods_packet_t periodsPacket;
     * if (kvh::KvhDeviceConfig::CreatePacketPeriodsPacket(packRequest, periodsPacket) != 0)
     *    // Error
     * // Use packet periods packet
     * @endcode
     */
    static int CreatePacketPeriodsPacket(KvhPacketRequest &_packetRequest, packet_periods_packet_t &);

    /**
     * @code
     * filter_options_packet_t filterOptions;
     * if (kvh::KvhDeviceConfig::CreateFilterOptionsPacket(filterOptions, ... // Additional options) != 0)
     *    // Error
     * // Use filter options
     * @endcode
     */
    static int CreateFilterOptionsPacket(
        filter_options_packet_t &,
        bool _permanent = true,
        uint8_t _vehicle_type = vehicle_type_car,
        bool _internal_gnss_enabled = true,
        bool _atmospheric_altitude_enabled = true,
        bool _velocity_heading_enabled = true,
        bool _reversing_detection_enabled = true,
        bool _motion_analysis_enabled = true);

    /**
     * @code
    * std::string port{'/dev/USB0'};
    * int curBaud = 115200;
    * int prevBaud = 9600;
    * kvh::KvhDeviceConfig::SetBaudRate(port, curBaud, prevBaud);
    * @endcode
    */
    static int SetBaudRate(std::string, int, int);

    /**
     * @code
     * kvh::KvhPacketRequest packetRequest = {
     *    // Create packet timer
     * }
     * kvh::KvhDeviceConfig::CalculateRequiredBaud(packetRequest);
     * @endcode
     */
    static int CalculateRequiredBaud(KvhPacketRequest&);
};

} // namespace kvh