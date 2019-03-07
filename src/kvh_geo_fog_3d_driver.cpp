/**
 * @file kvh_geo_fog_3d_driver.cpp
 * @brief KVH Geo Fog 3D driver class definitions.
 *
 * This is the class definitions file for the KVH Geo Fog 3D driver,
 * responsible for interfacing to the unit over the serial connection.
 */
#include "kvh_geo_fog_3d_driver.hpp"

kvh::Driver::Driver() :
  connected_(false)
{

} //end: Driver()

int kvh::Driver::Init()
{

} //end: Init()

int kvh::Driver::Once(kvh::MessageType* _messageType, std::vector<uint8_t>* _data)
{
  //Read serial port, check CRC16, check LRC, return data
  
} //end: Once()

int kvh::Driver::Cleanup()
{

} //end: Cleanup()
