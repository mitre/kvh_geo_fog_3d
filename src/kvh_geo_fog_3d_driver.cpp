/**
 * @file kvh_geo_fog_3d_driver.cpp
 * @brief KVH Geo Fog 3D driver class definitions.
 *
 * This file implements all functions defined in kvh_geo_fog_3d_driver.hpp. The driver 
 * is used for for interfacing with KVH GEO FOG over serial connection.
 */

//STD includes
#include <cstdio>
#include <string>
#include <cmath>
#include <functional>
#include <termios.h>
#include <typeinfo>
#include <set>
#include <iostream>
#include <fstream>

// RS232
#include "rs232.h"

// KVH Includes
#include "kvh_geo_fog_3d_driver.hpp"

#define RADIANS_TO_DEGREES (180.0 / M_PI)

namespace kvh
{

/**
 * @fn Driver::Driver
 * @brief Initializes connected status, port to use, and if debug printing is turned on.
 * 
 * @param _debug [in] Determines if debug statements are printed.
 */
Driver::Driver(bool _debug) :
  connected_(false),
  port_("/dev/ttyUSB0"),
  debug_(_debug)
{
} // END Driver()

/**
 * @fn Driver::~Driver
 * @brief Destructor. Will automatically cleanup the driver.
 */
Driver::~Driver()
{
  Cleanup();
} // END ~Driver()

// PRIVATE FUNCTIONS

/**
 * @fn Driver::DecodePacket
 * @param _anPacket [in] The packet to decode into a Kvh packet type
 * @param _packetMap [out] Map that the decoded packet will be placed in
 * 
 * @return [int]:
 *    0 = success,
 *    -1 = Unsupported packet type,
 *    -2 = Unable to decode packet
 */
int Driver::DecodePacket(an_packet_t *_anPacket, KvhPacketMap &_packetMap)
{

  // See if packet id is in our map
  if ((_packetMap.count(static_cast<packet_id_e>(_anPacket->id))) == 0)
  {
    // If packet is not in our map, print out the id and length and return as unsupported
    if (debug_) printf("Packet ID %u of Length %u\n", _anPacket->id, _anPacket->length);
    return -1;
  }

  // NOTICE: Code structure follows same pattern. Comments for first case work for all cases
  // The packet id is in our map, so decode the packet with the appropriate function
  // The list of supported packets below should match with Driver::CreatePacketMap
  if (_anPacket->id == packet_id_system_state) 
  {
    // copy all the binary data into the typedef struct for the packet 
    // this allows easy access to all the different values             
    if (decode_system_state_packet((system_state_packet_t *)_packetMap[packet_id_system_state].second.get(), _anPacket) == 0)
    {
      // Notify that we have updated packet
      _packetMap[packet_id_system_state].first = true;

      if (debug_)
      {
        system_state_packet_t system_state_packet = *(system_state_packet_t *)_packetMap[packet_id_system_state].second.get();
        printf("System State Packet:\n");
        printf("\tLatitude = %f, Longitude = %f, Height = %f\n", system_state_packet.latitude * RADIANS_TO_DEGREES, system_state_packet.longitude * RADIANS_TO_DEGREES, system_state_packet.height);
        printf("\tRoll = %f, Pitch = %f, Heading = %f\n", system_state_packet.orientation[0] * RADIANS_TO_DEGREES, system_state_packet.orientation[1] * RADIANS_TO_DEGREES, system_state_packet.orientation[2] * RADIANS_TO_DEGREES);
      }
    }
    else
    {
      if (debug_)
        printf("Failed to decode system state packet properly.\n");

      return -2;
    }
  }
  else if (_anPacket->id == packet_id_unix_time)
  {
    if (decode_unix_time_packet((unix_time_packet_t *)_packetMap[packet_id_unix_time].second.get(), _anPacket) == 0)
    {
      _packetMap[packet_id_unix_time].first = true;

      if (debug_)
      {
        unix_time_packet_t unix_time_packet = *(unix_time_packet_t *)_packetMap[packet_id_unix_time].second.get();
        printf("Unix Time Packet:\n");
        printf("Unix Time Seconds: %u, Unix Time Microseconds %u\n", unix_time_packet.unix_time_seconds, unix_time_packet.microseconds);
      }
    }
    else
    {
      if (debug_)
        printf("Failed to decode unix time packet properly.\n");

      return -2;
    }
  }
  else if (_anPacket->id == packet_id_raw_sensors)
  {
    if (decode_raw_sensors_packet((raw_sensors_packet_t *)_packetMap[packet_id_raw_sensors].second.get(), _anPacket) == 0)
    {
      _packetMap[packet_id_raw_sensors].first = true;

      if (debug_)
      {
        raw_sensors_packet_t raw_sensors_packet = *(raw_sensors_packet_t *)_packetMap[packet_id_raw_sensors].second.get();
        printf("Raw Sensors Packet:\n");
        printf("\tAccelerometers X: %f Y: %f Z: %f\n", raw_sensors_packet.accelerometers[0], raw_sensors_packet.accelerometers[1], raw_sensors_packet.accelerometers[2]);
        printf("\tGyroscopes X: %f Y: %f Z: %f\n", raw_sensors_packet.gyroscopes[0] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[1] * RADIANS_TO_DEGREES, raw_sensors_packet.gyroscopes[2] * RADIANS_TO_DEGREES);
      }
    }
    else
    {
      if (debug_)
        printf("Failed to decode raw sensors packet properly.\n");

      return -2;
    }
  }
  else if (_anPacket->id == packet_id_satellites)
  {
    if (decode_satellites_packet(static_cast<satellites_packet_t *>(_packetMap[packet_id_satellites].second.get()), _anPacket) == 0)
    {
      _packetMap[packet_id_satellites].first = true;
      if (debug_)
        printf("Collected satellites packet.\n");
    }
    else
    {
      if (debug_)
        printf("Failed to decode satellites packet properly.\n");
      
      return -2;
    }
  }
  else if (_anPacket->id == packet_id_satellites_detailed)
  {
    if (decode_detailed_satellites_packet(static_cast<detailed_satellites_packet_t *>(_packetMap[packet_id_satellites_detailed].second.get()), _anPacket) == 0)
    {
      _packetMap[packet_id_satellites_detailed].first = true;
      if (debug_)
        printf("Collected detailed satellites packet.\n");
    }
    else
    {
      if (debug_)
        printf("Failed to decode detailed satellites packet properly.\n");

      return -2;
    }
  }
  else if (_anPacket->id == packet_id_local_magnetics)
  {
    if (decode_local_magnetics_packet(static_cast<local_magnetics_packet_t *>(_packetMap[packet_id_local_magnetics].second.get()), _anPacket) == 0)
    {
      _packetMap[packet_id_local_magnetics].first = true;
      if (debug_)
        printf("Collected local magnetics packet.\n");
    }
    else
    {
      if (debug_)
        printf("Failed to decode local magnetics packet properly.\n");

      return -2;
    }
  }
  else if (_anPacket->id == packet_id_utm_position)
  {
    // Below we had to create a modified decode function since 
    /** \todo Fix utm_position_packet_t to follow data provided by kvh instead of what they have defined.*/

    if (DecodeUtmFix(static_cast<utm_fix *>(_packetMap[packet_id_utm_position].second.get()), _anPacket) == 0)
    {
      _packetMap[packet_id_utm_position].first = true;
      if (debug_)
        printf("Collected utm position packet.\n");
    }
    else
    {
       if (debug_)
        printf("Failed to decode utm position packet properly.\n");

      return -2;
    }
  }
  else if (_anPacket->id == packet_id_ecef_position)
  {
    if (decode_ecef_position_packet(static_cast<ecef_position_packet_t *>(_packetMap[packet_id_ecef_position].second.get()), _anPacket) == 0)
    {
      _packetMap[packet_id_ecef_position].first = true;
      if (debug_)
        printf("Collected ecef position packet.\n");
    }
    else
    {
      if (debug_)
        printf("Failed to decode ecef position packet properly.\n");

      return -2;
    }
  }
  else if (_anPacket->id == packet_id_north_seeking_status)
  { 
    if (decode_north_seeking_status_packet(static_cast<north_seeking_status_packet_t *>(_packetMap[packet_id_north_seeking_status].second.get()), _anPacket) == 0)
    {
      _packetMap[packet_id_north_seeking_status].first = true;
      if (debug_)
        printf("Collected north seeking status packet.\n");
    }
    else
    {
      if (debug_)
        printf("Failed to decode north seeking status packet properly.\n");

      return -2;
    }
  }
  else if (_anPacket->id == packet_id_euler_orientation_standard_deviation)
  {
    if (decode_euler_orientation_standard_deviation_packet(
      static_cast<euler_orientation_standard_deviation_packet_t*>(_packetMap[packet_id_euler_orientation_standard_deviation].second.get()), _anPacket) == 0)
    {
      _packetMap[packet_id_euler_orientation_standard_deviation].first = true;
      if (debug_) printf("Collected euler orientation standard deviation packet.");
    }
    else
    {
      if (debug_) printf("Failed to decode euler orientation standard devation packet.");
      return -2;
    }
  }

  return 0;
} // END DecodePacket()

int Driver::DecodeUtmFix(utm_fix* _utmPacket, an_packet_t* _anPacket)
{
	if(_anPacket->id == packet_id_utm_position && _anPacket->length == 26)
	{
		memcpy(&_utmPacket->position, &_anPacket->data[0], 3*sizeof(double));
		_utmPacket->zone_num = _anPacket->data[24];
    _utmPacket->zone = _anPacket->data[25];
		return 0;
	}
	else return 1;
}

/**
 * @fn Driver::SendPacket
 * @brief Wrapper function for more easily sending an packets via serial port
 * 
 * @param _anPacket [in] The an packet to transmit
 */
int Driver::SendPacket(an_packet_t *_anPacket)
{
  // Encode packet. Adds header including LRC and CRC
  an_packet_encode(_anPacket);
  // Send AN packet via serial port
  if (SendBuf(an_packet_pointer(_anPacket), an_packet_size(_anPacket)))
  {
    if (debug_)
      printf("Packet succesfully sent!\n");
    return 0;
    packetRequests_.push_back(static_cast<packet_id_e>(_anPacket->id));
  }
  else
  {
    if (debug_)
      printf("Unable to send packet.\n");
    return -1;
  }
} // END SendPacket()

// PUBLIC FUNCTIONS

/**
   * @fn Driver::Init
   * @brief Initialize the connection to the device
   * 
   * @param _port [in] Port the kvh is connected through
   * @param _packetsRequested [in] Vector of packet id's to ask the kvh to output
   * 
   * @return [int]: 0 = success, > 0 = warning, < 0 = failure
   * 
   * Initialize the serial connection to the KVH GEO FOG 3D.
   * 
   * Current calculation for our packets:
   * (105 (sys state) + 18 (satellites) +
   * (5+(7*(1 for min or 50 for max))) (Detailed satellites) + 17 (local mag)
   * + 30 (utm) + 29 (ecef) + 32) * rate (50hz default) * 11
   * Minimum baud all packets at 100hz for worst case scenario is 644600
   * 
   * @warning When connecting we make the massive assumption that the kvh starts
   * with a baud rate of 115200. The workflow here is Open comport -> Set baud to required
   * -> Close comport -> Open comport with baud required
   */
int Driver::Init(const std::string& _port, KvhPacketRequest _packetsRequested, bool _gnssEnabled)
{

  ///////////////////////////////////////
  // SETTING PACKET OUTPUT AND FREQUENCY
  // CALCULATE REQUIRED BAUDRATE BEFORE CONNECTING
  ///////////////////////////////////////

  packet_periods_packet_t packetPeriods;

  // Make permanent in case it has a hot reset, otherwise an error is likely
  packetPeriods.permanent = 1;
  // Clear all exisiting packet periods and replace with new ones
  packetPeriods.clear_existing_packets = 1;

  /* 
  * In the section below, we need to calculate the desired packet periods
  * given the desired packet rate (Hz) from the user. Additionally, we need to calculate
  * the necessary baud rate to make sure we have enough bandwidth for everything.
  * 
  * PACKET RATE -> PACKET PERIOD
  * Formula for how packet frequency relates to packet period:
  * Packet Rate = 1000000/(Packet Period * Packet Timer Period)Hz - From Manual
  * 
  * Packet Timer Period can also be set, but we will use its default of 1000us for now
  * So, given packet rate, the packet period we need to input is
  * 
  * 1000000/(Packet Rate * 1000) = Packet Period
  * 1000/Packet Rate = Packet Period
  * 
  * 
  * CALCULATING BAUDRATE:
  * 
  * Data throughput = (packet_length + 5 (for fixed packet overhead)) * rate
  * Minimum baud = data throughput * 11
  * -> Find closest baud
  * 
  */
  
  std::set<packet_id_e> packetIdList; // To hold list of already added packet id's
  int returnValue = 0; // Will hold number of duplicated id's

  int dataThroughput = 0;
  int i;
  for (i = 0; i < _packetsRequested.size(); i++)
  {
    std::pair<packet_id_e, int> packet = _packetsRequested.at(i);

    if (packetIdList.count(packet.first) > 0)
    {
      returnValue++; // Found duplicate, increase counter
    }

    // packet_period_t period = {packet id, period}
    packet_period_t period;
    period.packet_id = _packetsRequested.at(i).first;
    period.period = 1000/_packetsRequested.at(i).second; // Using formulate for rate to period derived above
    packetPeriods.packet_periods[i] = period;

    // Add this as part of our baudrate calculation
    // Increase required baudrate by (struct_size + 5) * rate
    dataThroughput += (_packetSize[packet.first] + 5) * packet.second;
  }

  ////////////////////////////////
  // SETTING BUAD RATE
  ////////////////////////////////

  //\todo Decision to make, we have calculated the required baud, should we try to be
  //\todo be close to it or should we pad for some extra space?
  //
  // I am going to pad my space for now by adding 5% on top
  // With the added padding we effectively operate at ~9000000 baud or below

  // dataThroughput from above, 11 from their equation, 1.05 as a buffering factor
  int minBaud = dataThroughput * 11 * 1.05;
  if (debug_) printf("Calculated baud rate: %d\n", minBaud);

  // Find smallest baud rate that will suffice
  if (minBaud > 10000000)
  {
    if (debug_) printf("Required baud rate too high!\n");
    return -1;
  }
  else if (minBaud > 921600)
  {
    baud_ = 10000000;
  }
  else if (minBaud > 576000)
  {
    baud_ = 921600;
  }
  else if (minBaud > 500000)
  {
    baud_ = 576000;
  }
  else if (minBaud > 460800)
  {
    baud_ = 500000;
  }
  else if (minBaud > 230400)
  {
    baud_ = 460800;
  }
  else if (minBaud > 115200)
  {
    baud_ = 230400;
  }
  else
  {
    baud_ = 115200;
  }

  if (debug_) printf("Baud set to: %d\n", baud_);

  // Testing
  // baud_ = 230400;

  baud_rates_packet_t baudRatePacket; 
  baudRatePacket.permanent = 1;
  baudRatePacket.primary_baud_rate = baud_;
  baudRatePacket.gpio_1_2_baud_rate = baud_;
  baudRatePacket.auxiliary_baud_rate = baud_;
  baudRatePacket.reserved = 0;


  ///////////////////////////////////////
  // SETTING UP KALMAN FILTER OPTIONS
  ///////////////////////////////////////

  filter_options_packet_t filterOptions;

	filterOptions.permanent = true;
	filterOptions.vehicle_type = vehicle_type_car;
	filterOptions.internal_gnss_enabled = _gnssEnabled; // Set if we want to test using gps or not
	filterOptions.atmospheric_altitude_enabled = true;
	filterOptions.velocity_heading_enabled = true;
	filterOptions.reversing_detection_enabled = true;
	filterOptions.motion_analysis_enabled = true;

  ////////////////////////////////////////
  // CONNECTING TO KVH
  ////////////////////////////////////////

  if (debug_) printf("Opening comport at 115200 baud.\n");

  port_ = _port;
  char portArr[4096];
  strncpy(portArr, port_.c_str(), 4096);
  ///////////////////////////////////////
  // MASSIVE ASSUMPTION
  // The kvh will always start with 115200 baud rate
  // We may change it, and then revert the change when 
  // the driver is exited
  //////////////////////////////////////
  if (OpenComport(portArr, 115200) != 0)
  {
    if (debug_) printf("Unable to establish connection.\n");
    return -1;
  }
  // We are connected to the KVH!
  connected_ = true;

  
  ////////////////////////////////
  // SENDING CONFIGURATION PACKETS
  ////////////////////////////////

  if (debug_) printf("Sending baud rate packet.\n");

  an_packet_t* requestPacket = encode_baud_rates_packet(&baudRatePacket);
  int packetError = SendPacket(requestPacket);
  an_packet_free(&requestPacket);
  requestPacket = nullptr;
  if (packetError){
    return -2;
  }

  // Need to close then reopen the comport
  CloseComport();
  connected_ = false;

  if (debug_) printf("Opening comport at %d baud\n", baud_);

  if (OpenComport(portArr, baud_) != 0)
  {
    if (debug_) printf("Unable to establish connection at %d baud.\n", baud_);
    return -1;
  }
  // We are connected to the KVH!
  connected_ = true;
  

  if (debug_) printf("Sending packet_periods.\n");

  requestPacket = encode_packet_periods_packet(&packetPeriods);
  packetError = SendPacket(requestPacket);
  an_packet_free(&requestPacket);
  requestPacket = nullptr;
  if (packetError){
    return -2;
  }

  if (debug_) printf("Sending filter options packet.");

  requestPacket = encode_filter_options_packet(&filterOptions);
  packetError = SendPacket(requestPacket);
  requestPacket = nullptr;
  if (packetError != 0)
  {
    return -2;
  }

  /////////////////////////////////////
  // INITIALISE AN DECODER
  /////////////////////////////////////

  if (debug_) printf("Initializing decoder.\n");
  an_decoder_initialise(&anDecoder_);

  return returnValue;

} // END Init()

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
   * 
   * \attention This function is a bit of a mess, due to how their api has a different function
   * for every single type of packet. Our goal is to be able to deal with all of them simply
   * within one single function, but that brings a lot of typing problems into the mix. Namely
   * making sure that we have the correct type of struct, and then calling the correct decoding
   * function for that struct. If there is a good way to pass around types, so that I could 
   * correctly cast that would be nice, but I do not know of a solution yet. Possibly using
   * decltype might work.
   * Assumptions:
   *  Since our map takes in a shared_ptr<void> to deal with having multiple types of structs
   * per function call, we have now way of knowing if they actually passed in the correct struct
   * type that matches the packet id. If they didn't I could easily see us running into a seg fault,
   * which might be disastrous. Perhaps I can at least check if the size of the struct we expect and
   * the size they passed in are the same? That might prevent seg faults but would still be open to other
   * errors.
   */
int Driver::Once(KvhPacketMap &_packetMap)
{
  // Set the updated value of each packet to false
  for (auto it = _packetMap.cbegin(); it != _packetMap.cend(); it++)
  {
    // Set all updates to false
    _packetMap[it->first].first = false;
  }

  an_packet_t *anPacket;
  int bytesRec;
  int unexpectedPackets = 0;

  // Check if new packets have been sent
  if ((bytesRec = PollComport(an_decoder_pointer(&anDecoder_), an_decoder_size(&anDecoder_))) > 0)
  {
    /* increment the decode buffer length by the number of bytes received */
    an_decoder_increment(&anDecoder_, bytesRec);

    /* decode all the packets in the buffer */
    while ((anPacket = an_packet_decode(&anDecoder_)) != NULL)
    {
      // If we get an acknowledgment packet from sending packets
      // Acknowledgement packet is different than the others so we keep it seperate
      if (anPacket->id == packet_id_acknowledge)
      {
        acknowledge_packet_t ackP;
        if (decode_acknowledge_packet(&ackP, anPacket) == 0)
        {
          if (debug_) {
            printf("*********************************\n"
            "Acknowledging packet from packet id: %d\n Result of packet %d\n"
            "********************************\n", ackP.packet_id, ackP.acknowledge_result);
          }
        }
        else
        {
          if (debug_) printf("Unable to decode acknowledge packet properly.\n");
        }
      }
      else
      {
        if (DecodePacket(anPacket, _packetMap) < 0)
          unexpectedPackets++;
      }

      /* Ensure that you free the an_packet when your done with it or you will leak memory */
      an_packet_free(&anPacket);
    }
  }
} // END Once()

/**
 * @fn Driver::CreatePacketMap
 * @brief Correctly sets up a KvhPacketMap for the requested packets
 * 
 * @return [int]:
 *    0 = Success,
 *    >0 = Warning. Warning number denotes number of unsupported packets passed in.
 */
int Driver::CreatePacketMap(KvhPacketMap &_packetMap, KvhPacketRequest _packRequest)
{
  int unsupported = 0;
  int i;
  for (i = 0; i < _packRequest.size(); i++)
  {
    packet_id_e packEnum = _packRequest.at(i).first;
    /*
     * General form for below:
     *  case (packetId):
     *    _packetMap[packetId] = pair(false, shared_ptr(packet_struct_t))
     */
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
      _packetMap[packet_id_utm_position] = std::make_pair(false, std::make_shared<utm_fix>());
      break;
    case packet_id_ecef_position:
      _packetMap[packet_id_ecef_position] = std::make_pair(false, std::make_shared<ecef_position_packet_t>());
      break;
    case packet_id_north_seeking_status:
      _packetMap[packet_id_north_seeking_status] = std::make_pair(false, std::make_shared<north_seeking_status_packet_t>());
      break;
    case packet_id_euler_orientation_standard_deviation:
      _packetMap[packet_id_euler_orientation_standard_deviation] = std::make_pair(false, std::make_shared<euler_orientation_standard_deviation_packet_t>());
      break;
    default:
      // If the packet id is not in the list above it is unsupported
      if (debug_)
        printf("Packet with id: %d unsupported", packEnum);
      unsupported += 1;
    }
  }

  // Will return 0 if we support all, or the number of entered id's we don't support if >0
  return unsupported;
} // END CreatePacketMap()

/**
   * @fn Driver::Cleanup
   * @brief Cleanup and close our connections.
   * @return [int]: 0 = success, > 0 = warning, < 0 = failure
*/
int Driver::Cleanup()
{
  // Before closing comport set the KVH back to 115200 baud
  baud_rates_packet_t baudRatePacket; 
  baudRatePacket.permanent = 1;
  baudRatePacket.primary_baud_rate = 115200;
  baudRatePacket.gpio_1_2_baud_rate = 115200;
  baudRatePacket.auxiliary_baud_rate = 115200;
  baudRatePacket.reserved = 0;

  if (debug_) printf("Sending baud rate packet before finishing up.\n");

  an_packet_t* requestPacket = encode_baud_rates_packet(&baudRatePacket);
  int packetError = SendPacket(requestPacket);
  an_packet_free(&requestPacket);
  requestPacket = nullptr;
  if (packetError){
    return -1;
  }

  CloseComport();
  return 0;
} // END Cleanup()

} // namespace kvh
