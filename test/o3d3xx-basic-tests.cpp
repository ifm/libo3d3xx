#include "o3d3xx.h"
#include <errno.h>
#include "gtest/gtest.h"

// exception mechanism
TEST(General_Tests, Exceptions)
{
  bool ex_caught = false;

  // example of throwing a library error
  try
    {
      throw(o3d3xx::error_t(O3D3XX_NO_ERRORS));
    }
  catch (const o3d3xx::error_t& ex)
    {
      ex_caught = true;
      EXPECT_EQ(O3D3XX_NO_ERRORS, ex.code());
      EXPECT_STREQ(o3d3xx::strerror(O3D3XX_NO_ERRORS), ex.what());
    }
  EXPECT_TRUE(ex_caught);
  ex_caught = false;

  // example of throwing a system error
  try
    {
      throw(o3d3xx::error_t(EAGAIN));
    }
  catch (const o3d3xx::error_t& ex)
    {
      ex_caught = true;
      EXPECT_EQ(EAGAIN, ex.code());
      EXPECT_STREQ(o3d3xx::strerror(EAGAIN), ex.what());
    }
  EXPECT_TRUE(ex_caught);
  ex_caught = false;
}

// versioning
TEST(General_Tests, Version)
{
  int major, minor, patch;

  o3d3xx::version(&major, &minor, &patch);
  EXPECT_EQ(O3D3XX_VERSION, O3D3XX_MAKE_VERSION(major, minor, patch));
}