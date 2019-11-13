/*
 * Author: George Karabassis
 * Co-Author: Beren Millidge
 * Organisation: HYPED
 * Date: 13/11/2019
 * Description: Get data from imus
 *
 *    Copyright 2018 HYPED
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


#include "imu_query.hpp"

using namespace hyped;
using namespace navigation;

IMU_querying::IMU_querying(int m)
  : m_(m)
  , dt_(0)
  , data_(Data::getInstance())
  , previous_timestamp_(0)
  , z_(m*data::Sensors::kNumImus)
{}

void IMU_querying::data_formatting()
{
  DataPoint<ImuDataArray> sensor_readings = data_.getSensorsImuData();

  for (int j = 0; j < m_; ++j) {
    for (int i = 0; i < data::Sensors::kNumImus; ++i) {
      // if faulty imu value, retrun 0 acceleration
      // else
      z_(j*data::Sensors::kNumImus + i) = sensor_readings.value[i].acc[j];
      // acc_raw[i] = a[axis_];  // accNorm(a) * (1 - 2 * (a[axis_] < 0));
    }
  }

  // outlier business here
}

void IMU_querying::dt()
{
  DataPoint<ImuDataArray> sensor_readings = data_.getSensorsImuData();

  uint32_t t = sensor_readings.timestamp;
  dt_ = t - previous_timestamp_;
  previous_timestamp_ = t;
}

VectorXf IMU_querying::get_measurement()
{
  return z_;
}

uint32_t IMU_querying::get_dt()
{
  return dt_;
}
