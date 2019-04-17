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

int Driver::DecodePacket(an_packet_t *_anPacket, KvhPackageMap &_packetMap)
{
  if (_anPacket->id == packet_id_acknowledge)
  {
    acknowledge_packet_t *ackP;
    if (decode_acknowledge_packet(ackP, _anPacket) == 0)
    {
      printf("Acknowledging packet from packet id: %d\n", ackP->packet_id);
    }
    else
    {
      if (verbose_)
        printf("Failed to decode system state packet properly.\n");
    }
    return 0; // Don't need to try the below for this packet
  }

  // Check if the packet id is in our map
  for (auto it = _packetMap.cbegin(); it != _packetMap.cend(); it++)
  {
    if (_anPacket->id == it->first)
    {
      // If the packet id is in our map, figure out which particular packet it is
      if (_anPacket->id == packet_id_system_state) /* system state packet */
      {
        /* copy all the binary data into the typedef struct for the packet */
        /* this allows easy access to all the different values             */
        if (decode_system_state_packet((system_state_packet_t *)_packetMap[packet_id_system_state].second.get(), _anPacket) == 0)
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
        else
        {
          if (verbose_)
            printf("Failed to decode system state packet properly.\n");
        }
      }
      else if (_anPacket->id == packet_id_unix_time)
      {
        if (decode_unix_time_packet((unix_time_packet_t *)_packetMap[packet_id_unix_time].second.get(), _anPacket) == 0)
        {
          _packetMap[packet_id_unix_time].first = true;

          if (verbose_)
          {
            unix_time_packet_t unix_time_packet = *(unix_time_packet_t *)_packetMap[packet_id_unix_time].second.get();
            printf("Unix Time Packet:\n");
            printf("Unix Time Seconds: %u, Unix Time Microseconds %u\n", unix_time_packet.unix_time_seconds, unix_time_packet.microseconds);
          }
        }
        else
        {
          if (verbose_)
            printf("Failed to decode unix time packet properly.\n");
        }
      }
      else if (_anPacket->id == packet_id_raw_sensors) /* raw sensors packet */
      {
        /* copy all the binary data into the typedef struct for the packet */
        /* this allows easy access to all the different values             */
        if (decode_raw_sensors_packet((raw_sensors_packet_t *)_packetMap[packet_id_raw_sensors].second.get(), _anPacket) == 0)
        {
          _packetMap[packet_id_raw_sensors].first = true;

          if (verbose_)
          {
            raw_sensors_packet_t raw_sensors_packet = *(raw_sensors_packet_t *)_packetMap[packet_id_raw_sensors].second.get();
            printf("Raw Sensors Packet:\n");
            printf("\tAccelerometers X: %f Y: %f Z: %f\n", raw_sensors_packet.accelerometers[0], raw_sensors_packet.accelerometers[1], raw_sensors_packet.accelerometers[2]);
            printf("\tGyroscopes X: %f Y: %f Z: %f\n", raw_sensors_packet.gyroscopes[0] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[1] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[2] * RADIANS_TO_DEGREES);
          }
        }
        else
        {
          if (verbose_)
            printf("Failed to decode raw sensors packet properly.\n");
        }
      }
      else if (_anPacket->id == packet_id_satellites)
      {
        if (decode_satellites_packet(static_cast<satellites_packet_t *>(_packetMap[packet_id_satellites].second.get()), _anPacket))
        {
          _packetMap[packet_id_satellites].first = true;
          if (verbose_)
            printf("Collected satellites packet.\n");
        }
        else
        {
          printf("Failed to decode satellites packet properly.\n");
        }
      }
      else if (_anPacket->id == packet_id_satellites_detailed)
      {
        if (decode_detailed_satellites_packet(static_cast<detailed_satellites_packet_t *>(_packetMap[packet_id_satellites_detailed].second.get()), _anPacket))
        {
          _packetMap[packet_id_satellites_detailed].first = true;
          if (verbose_)
            printf("Collected detailed satellites packet.\n");
        }
        else
        {
          if (verbose_)
            printf("Failed to decode detailed satellites packet properly.\n");
        }
      }
      else if (_anPacket->id == packet_id_local_magnetics)
      {
        if (decode_local_magnetics_packet(static_cast<local_magnetics_packet_t *>(_packetMap[packet_id_local_magnetics].second.get()), _anPacket))
        {
          _packetMap[packet_id_local_magnetics].first = true;
          if (verbose_)
            printf("Collected local magnetics packet.\n");
        }
        else
        {
          if (verbose_)
            printf("Failed to decode local magnetics packet properly.\n");
        }
      }
      else if (_anPacket->id == packet_id_utm_position)
      {
        if (decode_utm_position_packet(static_cast<utm_position_packet_t *>(_packetMap[packet_id_utm_position].second.get()), _anPacket))
        {
          _packetMap[packet_id_utm_position].first = true;
          if (verbose_)
            printf("Collected utm position packet.\n");
        }
        else
        {
          if (verbose_)
            printf("Failed to decode utm position packet properly.\n");
        }
      }
      else if (_anPacket->id == packet_id_ecef_position)
      {
        if (decode_ecef_position_packet(static_cast<ecef_position_packet_t *>(_packetMap[packet_id_ecef_position].second.get()), _anPacket))
        {
          _packetMap[packet_id_ecef_position].first = true;
          if (verbose_)
            printf("Collected ecef position packet.\n");
        }
        else
        {
          if (verbose_)
            printf("Failed to decode ecef position packet properly.\n");
        }
      }
      else if (_anPacket->id == packet_id_north_seeking_status)
      {
        if (decode_north_seeking_status_packet(static_cast<north_seeking_status_packet_t *>(_packetMap[packet_id_north_seeking_status].second.get()), _anPacket))
        {
          _packetMap[packet_id_north_seeking_status].first = true;
          if (verbose_)
            printf("Collected north seeking status packet.\n");
        }
        else
        {
          if (verbose_) printf("Failed to decode north seeking status packet properly.\n");
        }
      }
      else
      {
        printf("Packet ID %u of Length %u\n", _anPacket->id, _anPacket->length);
      }
    }
  }
}

// PUBLIC FUNCTIONS

/**
   * @fn Driver::Init
   * @brief Initialize the connection to the device
   * @return [int]: 0 = success, > 0 = warning, < 0 = failure
   * 
   * Initialize the serial connection to the KVH GEO FOG 3D.
   * TODO: Possibly add code to calculate baud rate?
   * 
   * Current calculation for our packets:
   * (105 (sys state) + 18 (satellites) +
   * (5+(7*(1 for min or 5 for max))) (Detailed satellites) + 17 (local mag)
   * + 30 (utm) + 29 (ecef) + 32) * rate (50hz default) * 11
   * Supposedly minimum baud should be 133650, but state
   * packet doesn't seem to be showing overflow?
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
// TODO: Do we need to request each packet every time?
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
    if (verbose_)
      printf("Adding request for: %d\n", it->first);
    requestPacket->data[i] = it->first;
    i++; // Increment package position

    // Set all updates to false
    _packetMap[it->first].first = false;
  }
  an_packet_encode(requestPacket);

  // Attempt to send our request packet
  if (SendBuf(an_packet_pointer(requestPacket), an_packet_size(requestPacket)))
  {
    if (verbose_)
      printf("Packet succesfully sent!\n");
  }
  else
  {
    if (verbose_)
      printf("We may have a problem.\n");
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
      DecodePacket(anPacket, _packetMap);

      /* Ensure that you free the an_packet when your done with it or you will leak memory */
      an_packet_free(&anPacket);
    }
  }
}

// Helper function to create map for users of driver
// TODO: Check that we support the id before adding, if we don't return a warning int (e.g. > 0)
int Driver::CreatePacketMap(KvhPackageMap &_packetMap, std::vector<packet_id_e> _packRequest)
{
  int unsupported = 0;
  for (packet_id_e &packEnum : _packRequest)
  {
    /**
     * General form for below:
     *  case (packetId):
     *    _packetMap[packetId] = pair(false, shared_ptr(packet_struct_t))
     **/
    switch (packEnum)
    {
    case packet_id_system_state:
      _packetMap[packet_id_system_state] = std::make_pair(false, std::make_shared<system_state_packet_t>());
      break;
    case packet_id_unix_time:
      _packetMap[packet_id_unix_time] = std::make_pair(false, std::make_shared<unix_time_packet_t>());
      break;
    case packet_id_raw_sensors:
      _packetMap[packet_id_raw_sensors] = std::make_pair(false, std::make_shared<raw_sensors_packet_t>());
      break;
    case packet_id_satellites:
      _packetMap[packet_id_satellites] = std::make_pair(false, std::make_shared<satellites_packet_t>());
      break;
    case packet_id_satellites_detailed:
      _packetMap[packet_id_satellites_detailed] = std::make_pair(false, std::make_shared<detailed_satellites_packet_t>());
      break;
    case packet_id_local_magnetics:
      _packetMap[packet_id_local_magnetics] = std::make_pair(false, std::make_shared<local_magnetics_packet_t>());
      break;
    case packet_id_utm_position:
      _packetMap[packet_id_utm_position] = std::make_pair(false, std::make_shared<utm_position_packet_t>());
      break;
    case packet_id_ecef_position:
      _packetMap[packet_id_ecef_position] = std::make_pair(false, std::make_shared<ecef_position_packet_t>());
      break;
    case packet_id_north_seeking_status:
      _packetMap[packet_id_north_seeking_status] = std::make_pair(false, std::make_shared<north_seeking_status_packet_t>());
    default:
      // If the packet id is not in the list above it is unsupported
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
