/*
 * Author:
 * Organisation: HYPED
 * Date:
 * Description: Main class for fake IMUs
 *
 *    Copyright 2019 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include <math.h>
#include <random>
#include <vector>
#include <algorithm>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>

#include "sensors/fake_imu.hpp"
#include "data/data_point.hpp"
#include "utils/timer.hpp"

namespace hyped {

using hyped::data::State;

namespace sensors {

FakeImu::FakeImu(utils::Logger& log,
                std::string acc_file_path,
                std::string dec_file_path,
                std::string em_file_path,
                bool is_fail,
                int fail_state,
                float noise)
    : log_(log),
      failure_happened_(false),
      noise_(noise),
      data_(data::Data::getInstance())
{
  acc_fail_[0] = -37.3942;
  acc_fail_[1] = 0;
  acc_fail_[2] = 9.8;
  acc_zero_[0] = 0;
  acc_zero_[1] = 0;
  acc_zero_[2] = 0;

  readDataFromFile(acc_file_path, dec_file_path, em_file_path);
  // TODO(anyone): log messages
  
}

void FakeImu::setFailure()
{
  // Random point of failure from 0 to 10 seconds
  // Generate a random time for a failure
  failure_time_ = (rand() % 10 + 1) * 1000000;
  
}

void FakeImu::getData(ImuData* data)
{
  // switch statement for states
  // call getAccValue

  int state = -1;

  State current_state = data_.getStateMachineData().current_state;
  switch (current_state) {
    case State::kCalibrating: state = 0;
    case State::kAccelerating: state = 1;
    case State::kNominalBraking: state = 2;
    case State::kEmergencyBraking: state = 2;
  }
  ImuData file_data = getAccValue(state);
  data = &file_data;

}

ImuData FakeImu::getAccValue(int state)
{
  // read from vector the current acc value given reference time
}


NavigationVector FakeImu::addNoiseToData(NavigationVector value, float noise)
{
  NavigationVector temp;
  static std::default_random_engine generator;

  for (int i = 0; i < 3; i++) {
    std::normal_distribution<NavigationType> distribution(value[i], noise);
    temp[i] = distribution(generator);
  }
  return temp;
}

void FakeImu::readDataFromFile(std::string acc_file_path,
                               std::string dec_file_path,
                               std::string em_file_path)
{
  for (int i = 0; i < 3; i++) {
    std::string file_path;
    uint32_t timestamp;
    std::vector<NavigationVector>* val_read;
    std::vector<bool>* bool_read;

    if (i == 0) {
      file_path = acc_file_path;
      timestamp = kAccTimeInterval;
      val_read  = &acc_val_read_;
      bool_read = &acc_val_operational_;
    } else if (i == 1) {
      file_path = dec_file_path;
      timestamp = kAccTimeInterval;
      val_read  = &dec_val_read_;
      bool_read = &dec_val_operational_;
    } else if (i == 2) {
      file_path = em_file_path;
      timestamp = kAccTimeInterval;
      val_read  = &em_val_read_;
      bool_read = &em_val_operational_;
    }
    std::ifstream file;
    file.open(file_path);
    if (!file.is_open()) {
      log_.ERR("Fake-IMU", "Wrong file path for argument: %d", i);
    }

    NavigationVector value;
    int counter = 0;
    uint32_t temp_time;
    std::string line;

    while (getline(file, line)) {
      std::stringstream input(line);
      input >> temp_time;

      // checks whether timestamp format matches refresh rate
      if (temp_time != timestamp*counter) {
        log_.ERR("Fake-IMU", "Timestamp format invalid %d", temp_time);
      }

      input >> value[0];
      value[1] = 0.0;
      value[2] = 9.8;

      val_read->push_back(addNoiseToData(value, noise_));
      bool_read->push_back(1);      // always true

      counter++;
    }

    file.close();
  }
}

bool FakeImu::accCheckTime()
{
  uint64_t now = utils::Timer::getTimeMicros();
  uint64_t time_span = (now - ref_time_) / 1000;

  if (time_span < kAccTimeInterval*acc_count_) {
    return false;
  }
  acc_count_ = time_span/kAccTimeInterval + 1;
  return true;
}

}}  // namespace hyped::sensors
