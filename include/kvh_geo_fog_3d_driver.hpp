/**
 * @file driver.hpp
 * @brief KVH Geo Fog 3D driver class header.
 *
 * This is the header file for the KVH Geo Fog 3D driver, responsible
 * for interfacing to the unit over the serial connection.
 */
#pragma once

namespace kvh
{
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
     * @return [int] 0 = success, > 0 = warning, < 0 = failure
     * 
     * Single data read.
     */
    int Once();
    /**
     * @brief Cleanup and close our connections.
     * @return [int] 0 = success, > 0 = warning, < 0 = failure
     */
    int Cleanup();
  private:
    bool connected_; ///< If we're connected to the localization unit
  }; //end: class Driver  
} //end: namespace kvh
