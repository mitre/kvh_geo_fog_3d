#pragma once

#include <map>
#include <set>

// GEO-FOG
#include "an_packet_protocol.h"
#include "spatial_packets.h"

namespace kvh
{
  extern std::set<packet_id_e> supportedPackets_; ///< Set of packets containing all packet_id's we support
  extern std::map<packet_id_e, int> packetSize_; ///< Map relating packet id's to the associated struct size. Used for baudrate calculation
  extern std::map<packet_id_e, std::string> packetTypeStr_; ///< Holds
  ///< the string value for the different types of structs.

  /**
   * @struct utm_fix
   * The kvh outputs the utm zone number, which their packet does not have.
   * We are inheriting from their packet and adding the field they left off.
   */
  struct utm_fix : utm_position_packet_t
  {
    uint8_t zone_num;
  };
}