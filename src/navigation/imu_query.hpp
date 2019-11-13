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

#ifndef NAVIGATION_IMU_QUERY_HPP_
#define NAVIGATION_IMU_QUERY_HPP_

#include <Eigen/Dense>

#include <cstdint>
#include <string>

#include <cstdio>
#include <fstream>

#include "data/data.hpp"
#include "data/data_point.hpp"
#include "sensors/imu.hpp"

using Eigen::MatrixXf;
using Eigen::VectorXf;

namespace hyped {

using data::Data;
using data::DataPoint;
using data::ImuData;
using utils::Logger;


namespace navigation {

class IMU_querying {
  public:
    typedef std::array<ImuData, data::Sensors::kNumImus>            ImuDataArray;
    typedef DataPoint<ImuDataArray>                                 ImuDataPointArray;

    explicit IMU_querying(int m);

    void data_formatting();

    void dt();

    void outlier_check();

    VectorXf get_measurement();

    uint32_t get_dt();

  private:
    int m_;
    uint32_t dt_;
    Data& data_;
    uint32_t previous_timestamp_;
    VectorXf z_;
};

}}

#endif  // NAVIGATION_IMU_QUERY_HPP_
