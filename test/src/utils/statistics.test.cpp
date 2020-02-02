/*
* Author: QA team
* Organisation: HYPED
* Date:
* Description:
*
*    Copyright 2019 HYPED
*    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
*    except in compliance with the License. You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
*    Unless required by applicable law or agreed to in writing, software distributed under
*    the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
*    either express or implied. See the License for the specific language governing permissions and
*    limitations under the License.
*/

#include "gtest/gtest.h"
#include "utils/logger.hpp"
#include "data/data.hpp"
#include "state_machine/hyped-machine.hpp"
#include "utils/math/statistics.hpp"

using hyped::data::Data;
using hyped::utils::math::RollingStatistics;

TEST(statsTest, initsToZero)
{
  hyped::utils::math::RollingStatistics<float> rs
     = hyped::utils::math::RollingStatistics<float>(10);
  ASSERT_EQ(rs.getMean(), 0);
  ASSERT_EQ(rs.getSum(), 0);
  ASSERT_EQ(rs.getVariance(), 0);
  ASSERT_EQ(rs.getStdDev(), 0);
}

TEST(statsTest, updateMoreThanWindowMovesWindow) 
{
  int window_size = 10;
  float val = 5;
  hyped::utils::math::RollingStatistics<float> rs = 
    hyped::utils::math::RollingStatistics<float>(window_size);
  for (int i = 0; i < window_size - 1; i += 1) {
    rs.update(val);
  }
  // check less than the window size 
  ASSERT_EQ(rs.getSum(), 45);
  ASSERT_EQ(rs.getMean(), 5);
  ASSERT_EQ(rs.getVariance(), 0);

  rs.update(10);
  // check the same window size
  ASSERT_EQ(rs.getSum(), 55);
  EXPECT_FLOAT_EQ(rs.getMean(), 5.5);
  EXPECT_FLOAT_EQ(rs.getVariance(), 2.5);

  rs.update(10);
  // Check this is still 60 not 65
  // If the window was larger than the window size, it will be 65 
  ASSERT_EQ(rs.getSum(), 60);
  EXPECT_FLOAT_EQ(rs.getMean(), 6);
  EXPECT_FLOAT_EQ(rs.getVariance(), 4.444444);
}
