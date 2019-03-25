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

#include "kvh_geo_fog_3d_driver.hpp"

/**
 * @fn kvh::Driver::Driver
 * @brief Default contstructor.
 */
kvh::Driver::Driver() : 
  connected_(false)
{

} //end: Driver()

// kvh::Driver::~Driver()
// {
//   Cleanup();
// }

// PRIVATE FUNCTIONS

/**
 * @fn kvh::Driver::GetDeviceIdString
 * @brief Will create a device id string using the manufacturer, product type, and serial number
 * 
 * @attention The device string is not necessarily unique, it is just combining several
 * identifying factors in an attempt to be as unique as possible so that we can correctly identify
 * the device we wish to interact with. It is possible for their still to be naming conflicts.
 * 
 * @param _returnString [out] Holder for the device id string
 * @param _handle [in] Handle to the device to get the string for
 * @param _desc [in] Device descriptor used to retrieve information
 * @return [int]:
 *  0 = Success,
 *  -1 = Unable to retrieve manufacturer,
 *  -2 = Unable to retrieve product type,
 *  -3 = Unable to retrieve serial number
 */
int kvh::Driver::GetDeviceIdString(std::string &_returnString, libusb_device_handle *_handle,
                                   libusb_device_descriptor &_desc)
{
  // In case string is already populated
  _returnString.clear();
  std::string deviceIdString;
  uint8_t string[256];
  int ret;

  // Create unique usb string from iManufacturer + iProduct + iSerial
  if (_desc.iManufacturer)
  {
    ret = libusb_get_string_descriptor_ascii(_handle, _desc.iManufacturer, string, sizeof(string));
    if (ret > 0)
      deviceIdString += (char *)string;
    else
      return -1;
  }

  deviceIdString += " - ";

  if (_desc.iProduct)
  {
    ret = libusb_get_string_descriptor_ascii(_handle, _desc.iProduct, string, sizeof(string));
    if (ret > 0)
      deviceIdString += (char *)string;
    else
      return -2;
  }

  deviceIdString += " - ";

  if (_desc.iSerialNumber)
  {
    ret = libusb_get_string_descriptor_ascii(_handle, _desc.iSerialNumber, string, sizeof(string));
    if (ret > 0)
      deviceIdString += (char *)string;
    else
      return -3;
  }

  _returnString = deviceIdString;

  // printf("Description: %s\n", description.c_str());
  return 0;
}

/**
 * @fn kvh::Driver::GetDeviceHandle
 * @brief Will get a handle to a kvh geo fog device, if one is plugged in and permissions are set correctly
 * 
 * @attention In order to get a handle to a device, you must first have the correct permissions. 
 * Here were the steps I had to take to get the correct permission:
 *  1. lsusb -t to find bus and dev number (see images findkvh)
 *  2. cd /dev/bus/usb/{bus num}
 *  3. sudo chmod a+rwx 00{dev number}
 * In my case I was bus 3, and the device number changes each time I would plug it in, so I would have to redo these steps.
 * 
 * @param _returnHandle [out] Returned handle to the kvh geo fog device if one exists
 * @return [int]:
 *  0 = Success,
 *  -1 = Unable to get device list,
 *  -2 = Failed to find correct handle
 *  
 */  
int kvh::Driver::GetDeviceHandle(libusb_device_handle *_returnHandle)
{
  libusb_device **devs;
  ssize_t count;

  count = libusb_get_device_list(NULL, &devs);
  if (count < 0)
  {
    return -1;
  }

  printf("Number of found devices: %d\n", (int)count);

  // Variables for connecting to usb
  struct libusb_device_descriptor desc;
  libusb_device_handle *handle = NULL;
  std::string deviceIdString;
  int ret;

  for (int i = 0; devs[i]; i++)
  {
    // Get device descriptor
    // Struct fields http://libusb.sourceforge.net/api-1.0/structlibusb__device__descriptor.html
    // Can get usb's on terminal using (sudo) lsusb (-v)
    ret = libusb_get_device_descriptor(devs[i], &desc);
    if (ret < 0)
    {
      printf("Failed to get device descriptor for device %d.", i);
    }

    // Get handle to usb device
    // http://libusb.sourceforge.net/api-1.0/group__libusb__dev.html#ga7df95821d20d27b5597f1d783749d6a4
    ret = libusb_open(devs[i], &handle);

    if (LIBUSB_SUCCESS == ret)
    {
      printf("Able to connect. Getting id string...\n");
      ret = GetDeviceIdString(deviceIdString, handle, desc);
      if (ret >= 0)
      {
        // Compare device id strings to see if identical
        if (kvhDeviceIdString_ == deviceIdString)
        {
          // This devices id string matches what we expect, return the handle
          _returnHandle = handle;
          printf("Found correct device...\n");
          return 0;
        }
      }
      else
      {
        printf("Error getting string.\n");
      }

      // If the device was incorrect, close the handle
      if (handle)
      {
        printf("Closing handle.\n");
        libusb_close(handle);
      }
    }
  }

  printf("Freeing device list.\n");
  libusb_free_device_list(devs, 1);
  return -2; // No matching device found
}

// PUBLIC FUNCTIONS

/**
 * @fn kvh::Driver::Init
 * @brief Initialize the connection to the device
 * @return [int]: 0 = success, > 0 = warning, < 0 = failure
 * 
 * Initialize the serial connection to the KVH GEO FOG 3D.
 */
int kvh::Driver::Init()
{
  int ret;

  // Needs to be called before any other libusb functions are used
  ret = libusb_init(NULL);
  if (ret < 0)
  {
    return ret;
  }

  ret = GetDeviceHandle(kvhHandle_);
  if (ret < 0)
    printf("Error getting handle.\n");
  else
    printf("Success getting handle.\n");

} //end: Init()

/**
 * @fn kvh::Driver::Once
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
int kvh::Driver::Once(kvh::MessageType *_messageType, std::vector<uint8_t> *_data)
{
  //Read serial port, check CRC16, check LRC, return data

} //end: Once()


/**
 * @fn kvh::Driver::Cleanup
 * @brief Cleanup and close our connections.
 * @return [int]: 0 = success, > 0 = warning, < 0 = failure
 */
int kvh::Driver::Cleanup()
{
  if (kvhHandle_)
  {
    libusb_close(kvhHandle_);
  }

  libusb_exit(NULL);
  return 0;
} //end: Cleanup()
