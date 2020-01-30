/*
* Author: George Karabassis
* Organisation: HYPED
* Date:
* Description: Testing Kalman Filter based on dummy data.
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
// #include "gmock/gmock.h"
#include "navigation/main.cpp"
#include "navigation/kalman_filter.cpp"
#include "navigation/navigation.cpp"
#include "navigation/IMUs_faker.cpp"
#include "utils/logger.hpp"

struct KalmanFilterTest : public ::testing::Test {
  protected:
    // Add here any types you wish to use across your classes
    hyped::utils::Logger _log;
    hyped::navigation::KalmanFilter KF_ = hyped::navigation::KalmanFilter(3, 1, 0);
    hyped::navigation::Navigation nav_ = hyped::navigation::Navigation(_log, 3, 1, 0, KF_);
    hyped::navigation::Main main_ = hyped::navigation::Main(_log, 3, 1, 0, nav_, KF_);

    void SetUp()
    {
      // Here define the properties of any new types
      // KF_->KalmanFilter(3, 1, 0);
      // nav_->Navigation(_log, 3, 1, 0, KF_);
      // main_->Main(_log, 3, 1, 0, nav_, KF_);
    }
    void TearDown() {}
};

// Tests which belong to a test fixture are labeled TEST_F
TEST_F(KalmanFilterTest, Kalman_filter_test)
{
  // Any assertions or further setting up needed go here
  main_.run();
  // ASSERT_EQ(<argument1>, <argument2>);
}