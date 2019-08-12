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
#include <set>
#include <map>
#include <memory>
#include <functional>
#include <iostream>

// GEO-FOG SDK
#include "rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

// Kvh Driver Components
#include "kvh_geo_fog_3d_packet_storage.hpp"
#include "kvh_geo_fog_3d_global_vars.hpp"
#include "kvh_geo_fog_3d_device_configuration.hpp"

namespace kvh
{
/**
   * @struct KvhInitOptions
   * Holds initialization options for the Kvh.
   * \todo Implement gnss enabled
   */
struct KvhInitOptions
{
  bool gnssEnabled{true};
  int baudRate{115200};
  bool debugOn{false};
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
  bool connected_;                  ///< True if we're connected to the localization unit.
  std::string port_;                ///< Port to connect to the kvh through. Ex. "/dev/ttyUSB0"
  const uint32_t PACKET_PERIOD{50}; ///< Setting for determining packet rate. **Note**: Does not
      ///< equal packet frequency. See manual for equation on how Packet Period affects packet rate.
  an_decoder_t anDecoder_;                  ///< Decoder for decoding incoming AN encoded packets.
  bool debug_{false};                       ///< Set true to print debug statements
  std::vector<packet_id_e> packetRequests_; ///< Keeps a list of packet requests we have sent that we should recieve
      ///< achknowledgement packets for. Add to list in SendPacket, remove in Once (this may cause a time delay
      ///< problem where the packet is already gone by the time this function is called)
  KvhInitOptions defaultOptions_;  ///< If no init options are passed in, use this as the default
  KvhPacketStorage packetStorage_; ///< Class responsible for handling packets and ensuring consistency
  KvhDeviceConfig deviceConfig_;   ///< Class responsible for configuring kvh geo fog

  // Private functions, see implementation for detailed comments
  int DecodePacket(an_packet_t *);
  int DecodeUtmFix(utm_fix *, an_packet_t *); // Special decode since their utm api is incorrect
  int SendPacket(an_packet_t *);

public:
  Driver(bool _debug = false);
  ~Driver();

  /**
   * \code
   * std::vector<packet_id_e> packetRequest{packet_id_system_state, packet_id_satellites}; 
   * std::string kvhPort("/dev/ttyUSB0");
   * kvh::Driver kvhDriver;
   * kvhDriver.Init(kvhPort, packetRequest);  
   * \endcode
   */
  int Init(const std::string &_port, KvhPacketRequest &);

  /**
   * \code
   * std::vector<packet_id_e> packetRequest{packet_id_system_state, packet_id_satellites}; 
   * std::string kvhPort("/dev/ttyUSB0");
   * kvh::KvhInitOptions initOptions;
   * initOptions.baud = 230600;
   * ... // Aditional options
   * kvh::Driver kvhDriver;
   * kvhDriver.Init(kvhPort, packetRequest, initOptions);  * 
   * \endcode
   */
  int Init(const std::string &_port, KvhPacketRequest &, KvhInitOptions _initOptions);

  /**
   * @code 
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
   * @endcode
   */
  int Once();

  bool PacketIsUpdated(packet_id_e);
  int SetPacketUpdated(packet_id_e, bool);

  template <class T>
  int GetPacket(packet_id_e _packetId, T &packet)
  {
    return packetStorage_.GetPacket(_packetId, packet);
  }

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
