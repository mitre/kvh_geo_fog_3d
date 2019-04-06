/**
 * @file kvh_geo_fog_3d_driver.cpp
 * @brief KVH Geo Fog 3D driver class definitions.
 *
 * This is the class definitions file for the KVH Geo Fog 3D driver,
 * responsible for interfacing to the unit over the serial connection.
 */
//STD includes
#include <cstdio>
#include <string>
#include <cmath>
#include <functional>
#include <termios.h>
#include <typeinfo>

// RS232
#include "rs232.h"

#include "kvh_geo_fog_3d_driver.hpp"

#define RADIANS_TO_DEGREES (180.0 / M_PI)

namespace kvh
{

/**
   * @fn Driver::Driver
   * @brief Default contstructor.
   */
Driver::Driver(bool verbose) : connected_(false),
                   port_("/dev/ttyUSB0"),
                   verbose_(verbose)
{
} //end: Driver()

Driver::~Driver()
{
  Cleanup();
}

// PRIVATE FUNCTIONS

// PUBLIC FUNCTIONS

/**
   * @fn Driver::Init
   * @brief Initialize the connection to the device
   * @return [int]: 0 = success, > 0 = warning, < 0 = failure
   * 
   * Initialize the serial connection to the KVH GEO FOG 3D.
   */
int Driver::Init()
{
  // Open Comport
  // Make these class variables
  printf("Opening comport\n");
  if (OpenComport(port_, baud_) != 0)
  {
    printf("Unable to establish connection.\n");
    return -1;
  }
  connected_ = true;

  printf("Initializing decoder.");
  an_decoder_initialise(&anDecoder_);

} //end: Init()

/**
   * @fn Driver::Once
   * @brief Do a single data read from the device
   * @param _messageType [out] The type of message read
   * @param _data [out] An array containing the read data
   * 
   * @return [int]:
   *   0  = success
   *   -1 = CRC16 failure
   *   -2 = LRC failure
   * 
   * Single data packet read.
   */
/**
   * This function is a bit of a mess, due to how their api has a different function
   * for every single type of packet. Our goal is to be able to deal with all of them simply
   * within one single function, but that brings a lot of typing problems into the mix. Namely
   * making sure that we have the correct type of struct, and then calling the correct decoding
   * function for that struct. If there is a good way to pass around types, so that I could 
   * correctly cast that would be nice, but I do not know of a solution yet. Possibly using
   * decltype might work.
   * 
   * Assumptions:
   *  Since our map takes in a shared_ptr<void> to deal with having multiple types of structs
   * per function call, we have now way of knowing if they actually passed in the correct struct
   * type that matches the packet id. If they didn't I could easily see us running into a seg fault,
   * which might be disastrous. Perhaps I can at least check if the size of the struct we expect and
   * the size they passed in are the same? That might prevent seg faults but would still be open to other
   * errors.
   */
// TODO: Probably split into multiple functions: Request packets, Receive, Decode
int Driver::Once(KvhPackageMap &_packetMap)
{
  // Request packets
  // Have to build our request packet since the api function only allows a request of
  // one at a time
  an_packet_t *requestPacket = an_packet_allocate(_packetMap.size(), packet_id_request);
  int i = 0;
  for (auto it = _packetMap.cbegin(); it != _packetMap.cend(); it++)
  {
    // Add to requests
    // printf("Adding request for: %d\n", it->first);
    requestPacket->data[i] = it->first;
    i++; // Increment package position

    // Set all updates to false
    _packetMap[it->first].first = false;
  }
  an_packet_encode(requestPacket);

  // Attempt to send our request packet
  if (SendBuf(an_packet_pointer(requestPacket), an_packet_size(requestPacket)))
  {
    if (verbose_) printf("Packet succesfully sent!\n");
  }
  else
  {
    if (verbose_) printf("We may have a problem.\n");
  }
  an_packet_free(&requestPacket);
  requestPacket = nullptr;

  an_packet_t *anPacket;
  int bytesRec;

  if ((bytesRec = PollComport(an_decoder_pointer(&anDecoder_), an_decoder_size(&anDecoder_))) > 0)
  {
    /* increment the decode buffer length by the number of bytes received */
    an_decoder_increment(&anDecoder_, bytesRec);

    /* decode all the packets in the buffer */
    while ((anPacket = an_packet_decode(&anDecoder_)) != NULL)
    {
      if (anPacket->id == packet_id_acknowledge)
      {
        acknowledge_packet_t *ackP;
        if (decode_acknowledge_packet(ackP, anPacket) == 0)
        {
          printf("Acknowledging packet from packet id: %d\n", ackP->packet_id);
        }
        continue; // Don't need to try the below for this packet
      }

      // Check if the packet id is in our map
      for (auto it = _packetMap.cbegin(); it != _packetMap.cend(); it++)
      {
        if (anPacket->id == it->first)
        {
          // If the packet id is in our map, figure out which particular packet it is
          if (anPacket->id == packet_id_system_state) /* system state packet */
          {
            /* copy all the binary data into the typedef struct for the packet */
            /* this allows easy access to all the different values             */
            if (decode_system_state_packet((system_state_packet_t *)_packetMap[packet_id_system_state].second.get(), anPacket) == 0)
            {
              // Notify that we have updated packet
              _packetMap[packet_id_system_state].first = true;

              if (verbose_)
              {
                system_state_packet_t system_state_packet = *(system_state_packet_t *)_packetMap[packet_id_system_state].second.get();
                printf("System State Packet:\n");
                printf("\tLatitude = %f, Longitude = %f, Height = %f\n", system_state_packet.latitude * RADIANS_TO_DEGREES, system_state_packet.longitude * RADIANS_TO_DEGREES, system_state_packet.height);
                printf("\tRoll = %f, Pitch = %f, Heading = %f\n", system_state_packet.orientation[0] * RADIANS_TO_DEGREES, system_state_packet.orientation[1] * RADIANS_TO_DEGREES, system_state_packet.orientation[2] * RADIANS_TO_DEGREES);
              }
            }
          }
          else if (anPacket->id == packet_id_unix_time)
          {
            if (decode_unix_time_packet((unix_time_packet_t *)_packetMap[packet_id_unix_time].second.get(), anPacket) == 0)
            {
              _packetMap[packet_id_unix_time].first = true;

              if (verbose_)
              {
                unix_time_packet_t unix_time_packet = *(unix_time_packet_t *)_packetMap[packet_id_unix_time].second.get();
                printf("Unix Time Packet:\n");
                printf("Unix Time Seconds: %u, Unix Time Microseconds %u\n", unix_time_packet.unix_time_seconds, unix_time_packet.microseconds);
              }
            }
          }
          else if (anPacket->id == packet_id_raw_sensors) /* raw sensors packet */
          {
            /* copy all the binary data into the typedef struct for the packet */
            /* this allows easy access to all the different values             */
            if (decode_raw_sensors_packet((raw_sensors_packet_t *)_packetMap[packet_id_raw_sensors].second.get(), anPacket) == 0)
            {
              _packetMap[packet_id_raw_sensors].first = true;

              if (verbose_)
              {
                raw_sensors_packet_t raw_sensors_packet = *(raw_sensors_packet_t *) _packetMap[packet_id_raw_sensors].second.get();
                printf("Raw Sensors Packet:\n");
                printf("\tAccelerometers X: %f Y: %f Z: %f\n", raw_sensors_packet.accelerometers[0], raw_sensors_packet.accelerometers[1], raw_sensors_packet.accelerometers[2]);
                printf("\tGyroscopes X: %f Y: %f Z: %f\n", raw_sensors_packet.gyroscopes[0] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[1] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[2] * RADIANS_TO_DEGREES);
              }
            }
          }
          else
          {
            printf("Packet ID %u of Length %u\n", anPacket->id, anPacket->length);
          }
        }
      }

      /* Ensure that you free the an_packet when your done with it or you will leak memory */
      an_packet_free(&anPacket);
    }
  }
}

// Helper function to create map for users of driver
// TODO: Check that we support the id before adding, if we don't return a warning int (e.g. > 0)
int Driver::CreatePacketMap(KvhPackageMap &_packMap, std::vector<packet_id_e> _packRequest)
{
  int unsupported = 0;
  for (packet_id_e &packEnum : _packRequest)
  {
    bool updated = false;

    switch (packEnum)
    {
    case packet_id_system_state:
      _packMap[packet_id_system_state] = std::make_pair(updated, std::make_shared<system_state_packet_t>());
      break;
    case packet_id_unix_time:
      _packMap[packet_id_unix_time] = std::make_pair(updated, std::make_shared<unix_time_packet_t>());
      break;
    case packet_id_raw_sensors:
      _packMap[packet_id_raw_sensors] = std::make_pair(updated, std::make_shared<raw_sensors_packet_t>());
      break;
    default:
      unsupported += 1;
    }
  }

  // Will return 0 if we support all, or the number of entered id's we don't support if >0
  return unsupported;
}

/**
   * @fn Driver::Cleanup
   * @brief Cleanup and close our connections.
   * @return [int]: 0 = success, > 0 = warning, < 0 = failure
*/
int Driver::Cleanup()
{
  CloseComport();
  return 0;
} //end: Cleanup()

} // namespace kvh
