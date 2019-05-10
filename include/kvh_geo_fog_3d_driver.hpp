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
  int baud_{115200}; ///< Baud rate for communicating with kvh
  const uint32_t PACKET_PERIOD{50}; ///< Setting for determining packet rate. **Note**: Does not
    ///< equal packet frequency. See manual for equation on how Packet Period affects packet rate. 
  an_decoder_t anDecoder_; ///< Decoder for decoding incoming AN encoded packets.
  bool debug_{false}; ///< Set true to print debug statements
  std::vector<packet_id_e> packetRequests_; ///< Keeps a list of packet requests we have sent that we should recieve 
    ///< achknowledgement packets for. Add to list in SendPacket, remove in Once (this may cause a time delay 
    ///< problem where the packet is already gone by the time this function is called) 

  // Private functions, see implementation for detailed comments
  int DecodePacket(an_packet_t*, KvhPacketMap&);
  int SendPacket(an_packet_t*);

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
  int Init(const std::string& _port, std::vector<packet_id_e>);

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
  int CreatePacketMap(KvhPacketMap&, std::vector<packet_id_e>);

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
