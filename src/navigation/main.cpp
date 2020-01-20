/*
 * Author: George Karabassis
 * Co-Author: 
 * Organisation: HYPED
 * Date: 05/04/2019
 * Description: Main algorithm.
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

#include <iostream>
#include "navigation/main.hpp"
#include "navigation/kalman_filter.hpp"
#include "navigation/IMUs_faker.hpp"
#include "navigation/kalman_filter.hpp"
#include "utils/timer.hpp"

namespace hyped {
namespace navigation {

  Main::Main(Logger& log, int n, int m, int k, Navigation& nav_, KalmanFilter& KF_)
    : log_(log)
    , sys_(System::getSystem())
    , nav_(nav_)
    , KF_(KF_)
    , dt_(0)
    , m_(m)
    , n_(n)
    , k_(k)
  {}

  void Main::run()
  {
    Main::initKF();  // initialize Kalman Filter

    bool navComplete = false;
    int i = 0;  // index per Kalman Filter calculation

    while (navComplete == false && sys_.running_) {
      // navigate
      if (Main::callibration() == true) {
        // callibration is successfull
        if (
          Main::getState() == State::kIdle || 
          Main::getState() == State::kReady)
        {
          break;
        } else if (Main::getState() == State::kCalibrating) {
          // ** I need to find out what the below code does ***
          nav_.calibrateGravity();
        } else if (Main::getState() == State::kAccelerating) {
          nav_.initTimestamps();
        } else if (
          Main::getState() == State::kNominalBraking ||
          Main::getState() == State::kEmergencyBraking ||
          Main::getState() == State::kExiting ||
          Main::getState() == State::kRunComplete)
        {
          Main::navigate(i);
          i++;
        } else {
          navComplete = true;
          break;
        }
      }
    }
  }

  bool Main::callibration()
  {
    // TODO(george): Complete.
    return true;
  }

  void Main::initKF()
  {
    MatrixXf R(m_, m_);
    R << 0.0035, 0.00532,
          0.00125, 0.00780;

    MatrixXf P(n_, n_);
    P << 0.333, 0.54, 0.5444, 0.312, 0.54, 0.09844,
        0.53, 0.41, 0.122, 0.22, 0.51, 0.12342,
        0.6, 0.3, 0.23, 0.236, 0.4513, 0.1233,
        0.431, 0.52, 0.23, 0.546, 0.651, 0.33,
        0.261, 0.43, 0.73, 0.536, 0.153, 0.433,
        0.764, 0.12, 0.53, 0.483, 0.553, 0.133;

    MatrixXf Q(n_, n_);
    Q << 0.021, 0.022, 0.0243, 0.002, 0.0032, 0.0023,
        0.0242, 0.05322, 0.0214, 0.012, 0.0052, 0.023,
        0.0132, 0.0074, 0.0123, 0.00043, 0.0022, 0.012,
        0.0432, 0.0092, 0.0213, 0.000363, 0.0011, 0.042,
        0.0323, 0.02441, 0.053, 0.000732, 0.00315, 0.016,
        0.01312, 0.0224, 0.023, 0.000313, 0.00323, 0.072;

    VectorXf s(n_);
    s << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    KalmanFilter KF_ = KalmanFilter(n_, m_, k_);
    KF_.init(s, P, Q, R);
    KF_.setInitial(s);
  }

  void Main::navigate(int i)
  {
    nav_.run(i);
    // TODO(george): run navigation algorithm from Navigation class.
  }

  State Main::getState()
  {
    Data& data = Data::getInstance();
    State current_state = data.getStateMachineData().current_state;
    return current_state;
  }

  void Navigation::initTimestamps()
  {
    init_timestamp_ = utils::Timer::getTimeMicros();
    log_.DBG3("NAV", "Initial timestamp:%d", init_timestamp_);
    // TODO(george): Complete.
  }

}}  // namespace hyped navigation
