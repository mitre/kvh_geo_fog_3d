#include <gtest/gtest.h>
#include "kvh_geo_fog_3d_driver.hpp"
#include "init_vars.hpp"

#define GTEST_COUT std::cerr << "[   INFO   ] "

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new KvhPackReqEnv);
    return RUN_ALL_TESTS();
}