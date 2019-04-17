/**
 * @file driver.hpp
 * @brief KVH Geo Fog 3D driver class header.
 *
 * This is the header file for the KVH Geo Fog 3D driver, responsible
 * for interfacing to the unit over the serial connection.
 */
#pragma once

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

  typedef std::map<packet_id_e, std::pair<bool, std::shared_ptr<void>>> KvhPackageMap;
/**
   * @ingroup kvh
   * @brief Enumeration of KVH messages, based on the Packet ID.
   *
   * This enumeration should only include messages supported
   * by this interface.
   */
enum MessageType
{
  PACKET_SYSTEM_STATE = 20
}; //end: MessageType

// template<typename T>
// struct packetRequest
// {
//   T packet;
//   MessageType packetType;
// };

// Tripping points:
// Not all packets have encoding functions
// restore_factory_settings_packet and reset_packet encoding have no parameters
// template <typename T>
// struct PacketInfo
// {
//   packet_id_e packetId;
//   int (*decodeFn)(T *, an_packet_t *);
//   an_packet_t *(*encodeFn)(T *);

//   PacketInfo(packet_id_e packetId,
//              int (*decodeFn)(T *, an_packet_t *),
//              an_packet_t *(*encondFn)(T *) = nullptr) : packetId(packetId),
//                                                         decodeFn(decodeFn),
//                                                         encodeFn(encodeFn)
//   {
//   }
// };

/**
   * @class Driver
   * @ingroup kvh
   * @brief Driver worker class for the KVH Geo Fog 3D.
   */
class Driver
{
public:
  Driver(bool verbose = false);
  ~Driver();

  // Documentation here is about using interface. Implementation documentation in source

  int Init(std::vector<packet_id_e>);
  int Once(KvhPackageMap&);
  int CreatePacketMap(KvhPackageMap&, std::vector<packet_id_e>);
  int Cleanup();

private:
  bool connected_; ///< If we're connected to the localization unit
  char port_[13];
  int baud_{115200};
  const uint32_t PACKET_PERIOD{10};
  an_decoder_t anDecoder_;
  bool verbose_{false};

  int DecodePacket(an_packet_t*, KvhPackageMap&);
  // Map linking packet types to id's and their decoding and enconding functions
  // std::map<packet_id_e, std::shared_ptr<void>> packetInfoMap_ =
  // {
  //   {packet_id_system_state, std::make_shared<PacketInfo<system_state_packet_t>>(packet_id_system_state, decode_system_state_packet)}
  // };

}; //end: class Driver

} // namespace kvh
