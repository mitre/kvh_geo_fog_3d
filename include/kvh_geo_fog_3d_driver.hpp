/**
 * @file kvh_geo_fog_3d_driver.hpp
 * @brief KVH Geo Fog 3D driver class header.
 *
 * This is the header file for the KVH Geo Fog 3D driver, responsible
 * for interfacing to the unit over the serial connection.
 */
#pragma once

// STD
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <iostream>

// GEO-FOG SDK
#include "rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

namespace kvh
{
  /**
   * @typedef KvhPackageMap
   * Defines mapping from packet id's given in spatial_packets.h to a pair
   * that contains a pointer to a struct of the type represented by the id, and a boolean
   * denoting if the struct has been changed (for use when the map is passed to the
   * Driver::Once function).
   * 
   * @attention If possible, use Driver::CreatePacketMap function
   * to create and populate this map properly
   */
  typedef std::map<packet_id_e, std::pair<bool, std::shared_ptr<void>>> KvhPacketMap;

  typedef std::vector<std::pair<packet_id_e, int>> KvhPacketRequest;

  // Fixing utm packet
  struct utm_fix : utm_position_packet_t
  {
    uint8_t zone_num;
  };

/**
   * @class Driver
   * @ingroup kvh
   * @brief Driver worker class for the KVH Geo Fog 3D.
   * 
   * @todo Add functions for changing output packets and packet rate
   * @todo Make packet output rate variable
   */
class Driver
{
private:
  bool connected_; ///< True if we're connected to the localization unit.
  std::string port_; ///< Port to connect to the kvh through. Ex. "/dev/ttyUSB0"
  const uint32_t PACKET_PERIOD{50}; ///< Setting for determining packet rate. **Note**: Does not
    ///< equal packet frequency. See manual for equation on how Packet Period affects packet rate. 
  an_decoder_t anDecoder_; ///< Decoder for decoding incoming AN encoded packets.
  bool debug_{false}; ///< Set true to print debug statements
  std::vector<packet_id_e> packetRequests_; ///< Keeps a list of packet requests we have sent that we should recieve 
    ///< achknowledgement packets for. Add to list in SendPacket, remove in Once (this may cause a time delay 
    ///< problem where the packet is already gone by the time this function is called) 
  
  std::map<packet_id_e, int> _packetSize {
    {packet_id_system_state, sizeof(system_state_packet_t)},
    {packet_id_unix_time, sizeof(unix_time_packet_t)},
    {packet_id_raw_sensors, sizeof(raw_sensors_packet_t)},
    {packet_id_satellites, sizeof(satellites_packet_t)},
    {packet_id_satellites_detailed, sizeof(detailed_satellites_packet_t)},
    {packet_id_local_magnetics, sizeof(local_magnetics_packet_t)},
    {packet_id_utm_position, sizeof(utm_position_packet_t)},
    {packet_id_ecef_position, sizeof(ecef_position_packet_t)},
    {packet_id_north_seeking_status, sizeof(north_seeking_status_packet_t)},
    {packet_id_euler_orientation_standard_deviation, sizeof(euler_orientation_standard_deviation_packet_t)}
  }; ///< Map relating packet id's to the associated struct size. Used for baudrate calculation

  // Private functions, see implementation for detailed comments
  int DecodePacket(an_packet_t*, KvhPacketMap&);
  int DecodeUtmFix(utm_fix*, an_packet_t*); // Special decode since their utm api is incorrect
  int SendPacket(an_packet_t*);
  int BuildPacketPeriods(KvhPacketRequest, packet_period_t&);

  public:
  Driver(bool _debug = false);
  ~Driver();

  /**
   * \code
   * std::vector<packet_id_e> packetRequest{packet_id_system_state, packet_id_satellites}; 
   * std::string kvhPort("/dev/ttyUSB0");
   * kvh::Driver kvhDriver;
   * kvhDriver.Init(kvhPort, packetRequest);  * 
   * \endcode
   */
  int Init(const std::string& _port, KvhPacketRequest, int _baudRate = 115200, bool _gnssEnabled = true);

  /**
   * \code 
   * std::vector<packet_id_e> packetRequest{packet_id_system_state, packet_id_satellites};
   * KvhPacketMap packetMap;
   * kvh::Driver kvhDriver;
   * kvhDriver.CreatePacketMap(packetMap, packetRequest); // Populate map
   * kvhDriver.Once(kvhDriver); // Collect data
   * // Check if a packet was retrieved
   * if (packetMap[packet_id_system_state].first)
   * {
   *    system_state_packet_t sysPacket = *static_cast<system_state_packet_t *>(packetMap[packet_id_system_state].second.get());
   *    .... // Do stuff with the retrived packet
   * }
   */
  int Once(KvhPacketMap&);

  /**
   * \code
   * KvhPacketMap packetMap;
   * kvh::Driver kvhDriver;
   * // Populates map with packet id -> pair {boolean updated, pointer to struct}
   * // Ex packet_id_e_system_state -> {bool, system_state_packet*}
   * kvhDriver.CreatePacketMap(packetMap, packetRequest);
   * // To retrieve the struct:
   * system_state_packet_t sysPacket = *static_cast<system_state_packet_t *>(packetMap[packet_id_system_state].second.get());
   * \endcode
   */
  static int CreatePacketMap(KvhPacketMap&, KvhPacketRequest, bool _debug = false);

  /**
   * \code
   * kvh::Driver kvhDriver
   * // Initialize and use driver
   * kvhDriver.Cleanup();
   * \endcode
   */
  int Cleanup();

}; //end: class Driver

} // namespace kvh
