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
Driver::Driver() : connected_(false),
                   port_("/dev/ttyUSB0")
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
int Driver::Once(std::map<packet_id_e, std::pair<bool, std::shared_ptr<void>>> &_packetMap)
{
  // Request packets
  // an_packet_t *requestPacket = an_packet_allocate(_packetMap.size(), packet_id_request);
  // int i = 0;
  // for (auto it = _packetMap.cbegin(); it != _packetMap.cend(); it++)
  // {
  //   printf("Adding request for: %d\n", it->first);
  //   requestPacket->data[i] = it->first;
  //   i++;
  // }
  // an_packet_encode(requestPacket);

  // Attempt to send our request packet
  // printf("Size of packet: %d\n", (int)sizeof(*requestPacket));
  // if (SendBuf((unsigned char *)requestPacket, (int)sizeof(*requestPacket)) == 0)
  // {
  //   printf("Packet succesfully sent!\n");
  // }
  // else
  // {
  //   printf("We may have a problem.\n");
  // }
  // free(requestPacket);
  // requestPacket = nullptr;

  an_packet_t *anPacket;
  int bytesRec;

  if ((bytesRec = PollComport(an_decoder_pointer(&anDecoder_), an_decoder_size(&anDecoder_))) > 0)
  {
    /* increment the decode buffer length by the number of bytes received */
    an_decoder_increment(&anDecoder_, bytesRec);

    /* decode all the packets in the buffer */
    while ((anPacket = an_packet_decode(&anDecoder_)) != NULL)
    {

      //     if (anPacket->id == packet_id_acknowledge)
      //     {
      //       acknowledge_packet_t *ackP;
      //       if (decode_acknowledge_packet(ackP, anPacket) == 0)
      //       {
      //         printf("Acknowledging packet from packet id: %d\n", ackP->packet_id);
      //       }
      //       continue; // Don't need to try the below
      //     }

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
              system_state_packet_t system_state_packet = *(system_state_packet_t *)_packetMap[packet_id_system_state].second.get();
              printf("System State Packet:\n");
              printf("\tLatitude = %f, Longitude = %f, Height = %f\n", system_state_packet.latitude * RADIANS_TO_DEGREES, system_state_packet.longitude * RADIANS_TO_DEGREES, system_state_packet.height);
              printf("\tRoll = %f, Pitch = %f, Heading = %f\n", system_state_packet.orientation[0] * RADIANS_TO_DEGREES, system_state_packet.orientation[1] * RADIANS_TO_DEGREES, system_state_packet.orientation[2] * RADIANS_TO_DEGREES);
            }
          }
          // else if (anPacket->id == packet_id_raw_sensors) /* raw sensors packet */
          // {
          //   /* copy all the binary data into the typedef struct for the packet */
          //   /* this allows easy access to all the different values             */
          //   if (decode_raw_sensors_packet(&raw_sensors_packet, anPacket) == 0)
          //   {
          //     printf("Raw Sensors Packet:\n");
          //     printf("\tAccelerometers X: %f Y: %f Z: %f\n", raw_sensors_packet.accelerometers[0], raw_sensors_packet.accelerometers[1], raw_sensors_packet.accelerometers[2]);
          //     printf("\tGyroscopes X: %f Y: %f Z: %f\n", raw_sensors_packet.gyroscopes[0] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[1] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[2] * RADIANS_TO_DEGREES);
          //   }
          // }
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
