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
#include <string>

extern "C"
{
#include "libusb-1.0/libusb.h"
}

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
    // ~Driver();

    // Documentation here is about using interface. Implementation documentation in source

    int Init();
    int Once(kvh::MessageType* _messageType, std::vector<uint8_t>* _data);
    int Cleanup();

  private:
    const std::string kvhDeviceIdString_{"FTDI - USB-RS422 Cable - FT0NIR20"}; ///< Unique id string for kvh-geofog.
      ///< @attention Is not truly unique, so may run into name collisions with other kvh software, be aware.
    bool connected_; ///< If we're connected to the localization unit
    libusb_device_handle* kvhHandle_{NULL}; ///< Handle to the kvh geo fog device

    static int GetDeviceIdString(std::string&, libusb_device_handle *, libusb_device_descriptor&);
    int GetDeviceHandle(libusb_device_handle *);
  }; //end: class Driver  
} //end: namespace kvh
