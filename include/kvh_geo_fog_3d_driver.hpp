/**
 * @file driver.hpp
 * @brief KVH Geo Fog 3D driver class header.
 *
 * This is the header file for the KVH Geo Fog 3D driver, responsible
 * for interfacing to the unit over the serial connection.
 */
#pragma once

#include <vector>
#include <cstdint>

namespace kvh
{
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
  /**
   * @class Driver
   * @ingroup kvh
   * @brief Driver worker class for the KVH Geo Fog 3D.
   */
  class Driver
  {
  public:
    Driver();
    /**
     * @brief Initialize the connection to the device
     * @return [int] 0 = success, > 0 = warning, < 0 = failure
     * 
     * Initialize the serial connection to the KVH GEO FOG 3D.
     */
    int Init();
    /**
     * @brief Do a single data read from the device
     * @param _messageType [out] The type of message read
     * @param _data [out] An array containing the read data
     * @return [int]:
     *  0  = success
     *  -1 = CRC16 failure
     *  -2 = LRC failure
     * 
     * Single data packet read.
     */
    int Once(kvh::MessageType* _messageType, std::vector<uint8_t>* _data);
    /**
     * @brief Cleanup and close our connections.
     * @return [int] 0 = success, > 0 = warning, < 0 = failure
     */
    int Cleanup();
  private:
    bool connected_; ///< If we're connected to the localization unit
  }; //end: class Driver  
} //end: namespace kvh
