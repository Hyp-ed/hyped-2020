/*
 * Author: George Karabassis
 * Co-Author: 
 * Organisation: HYPED
 * Date: 05/04/2019
 * Description: Navigation algorithm.
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

#ifndef NAVIGATION_NAVIGATION_ALGORITHM_HPP_
#define NAVIGATION_NAVIGATION_ALGORITHM_HPP_

#include <array>
#include <cstdint>
#include <math.h>

#include "data/data.hpp"
#include "data/data_point.hpp"
#include "sensors/imu.hpp"
#include "navigation/kalman_filter.hpp"
#include "utils/logger.hpp"
#include "utils/math/integrator.hpp"
#include "utils/math/statistics.hpp"

namespace hyped {

using data::Data;
using data::DataPoint;
using data::ImuData;
using data::ModuleStatus;
using data::NavigationType;
using data::NavigationVector;
using data::Motors;
using navigation::KalmanFilter;
using utils::Logger;

namespace navigation {

class Navigation {
  public:
    typedef std::array<ImuData, data::Sensors::kNumImus>            ImuDataArray;
    typedef DataPoint<ImuDataArray>                                 ImuDataPointArray;
    Navigation(Logger& log, int n, int m, int k, KalmanFilter& KF_);
    // VectorXf getState();
    void initTimestamps();
    bool calibrateGravity();
    void run(int i);

  private:
    VectorXf IMUQuerying();
    void OutlierDetection();
    void KFCalc(int i, VectorXf z);
    void StackingKFCalc();
    void QueryKeyences();
    bool SensorsResultsSimilarity();
    void timestampUpdate();

    Logger& log_;
    System& sys_;
    Data& data_;
    KalmanFilter KF_;
    uint32_t init_timestamp_;  // current timestamp
    uint32_t prev_timestamp_;  // previous timestamp
    float dt_;
    int m_;
    int n_;
    int k_;
};

}}  // namespace hyped::navigation

#endif  // NAVIGATION_NAVIGATION_ALGORITHM_HPP_
