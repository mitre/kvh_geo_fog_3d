#pragma once

#include <gtest/gtest.h>
#include "kvh_geo_fog_3d_driver.hpp"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

class KvhPackReqEnv : public ::testing::Environment
{
public:
    typedef std::pair<packet_id_e, uint16_t> freqPair;

    static kvh::KvhPacketRequest zeroRequest;
    static kvh::KvhPacketRequest smallRequest;
    static kvh::KvhPacketRequest largeRequest;
    static kvh::KvhPacketRequest largeVaryingRatesRequest;
    static kvh::KvhPacketRequest minFrequencyRequest;
    static kvh::KvhPacketRequest maxFrequencyRequest;
    static kvh::KvhPacketRequest exceedingMaxFrequencyRequest;
    static kvh::KvhPacketRequest duplicateRequest;
    static kvh::KvhPacketRequest unsupportedRequest;
    static kvh::KvhPacketRequest unsupportedDuplicateRequest;
};