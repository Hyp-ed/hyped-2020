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
                bool is_fail,                     // these from config
                State fail_state,
                float noise)
    : log_(log),
      failure_happened_(false),
      is_fail_(is_fail),
      fail_state_(fail_state),
      noise_(noise),
      data_(data::Data::getInstance())
{
  acc_fail_.acc[0] = -37.3942;
  acc_fail_.acc[1] = 0;
  acc_fail_.acc[2] = 9.8;
  acc_fail_.operational = true;
  acc_zero_.acc[0] = 0;
  acc_zero_.acc[1] = 0;
  acc_zero_.acc[2] = 9.8;
  acc_zero_.operational = true;
  prev_acc_ = &acc_zero_;

  kFifoSize = 500;

  readDataFromFile(acc_file_path, dec_file_path, em_file_path);
  if (is_fail_) {
    log_.INFO("Fake-Imu", "Fake-Imu FAIL initialised");
  } else {
    log_.INFO("Fake-Imu", "Fake-Imu initialised");
  }
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

  // read_->clear();
  // time_->clear();
  State current_state = data_.getStateMachineData().current_state;

  if (current_state == State::kAccelerating) {
      read_ = &acc_val_read_;
      time_ = &acc_val_time_;
  } else if (current_state == State::kNominalBraking) {
      read_ = &dec_val_read_;
      time_ = &dec_val_time_;
  } else if (current_state == State::kEmergencyBraking) {
      read_ = &em_val_read_;
      time_ = &em_val_time_;
  }
  ref_time_ = utils::Timer::getTimeMicros();
  if (is_fail_) {
    if (fail_state_ == current_state) {     // if we are in current fail state
      fail_now_ = true;
    } else {
      fail_now_ = false;
    }  
  }
  if (current_state == State::kCalibrating) {        // stationary states
    data = &acc_zero_;
  } else {
    if (accCheckTime()) {
      ImuData file_data = getAccValue();
      prev_acc_ = &file_data;
      data = &file_data;
    } else {
      data = prev_acc_;
    }
  }
}

ImuData FakeImu::getAccValue()
{
  // read from vector the current acc value given reference time
  // iterate through timestamp vector to find correct time
  // get index and return acc value of that index in acc vector
  ImuData return_data;

  uint32_t current_time = utils::Timer::getTimeMicros();

  uint32_t vector_time;
  bool is_time = false;
  uint8_t count = 0;
  while (!is_time) {
    if (count >= read_->size()) {         // last value if out of bounds
      //vector_time = time_[count-1];
      vector_time = time_->at(count-1);
      is_time = true;                    // out of bounds
    } else {
      //vector_time = time_[count];
      vector_time = time_->at(count);
    }
    if (current_time - ref_time_ >= vector_time) {
      return_data.operational = true;
      return_data.acc = read_->at(count);
      is_time = true;
    }
    count++;
  }

  // re-noising acc value to fill FIFO
  for (int i = 0; i < kFifoSize; i++) {
    fifo_acc_[i] = addNoiseToData(return_data.acc, noise_);
  }

  // added loop below to temparily fix the error in the code
  for (int i = 0; i < kFifoSize; i++) {
    return_data.fifo[i] = fifo_acc_[i];
  }
  //return_data.fifo = fifo_acc_;

  if (fail_now_) {
    if (utils::Timer::getTimeMicros() - ref_time_ >= failure_time_ || failure_happened_) {
      if (!failure_happened_) {
        log_.INFO("Fake-IMU", "Start failure...");
      }
      return_data = acc_fail_;
      failure_happened_ = true;
    }
  }
  return return_data;
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
    std::vector<uint32_t>* timestamp;
    std::vector<NavigationVector>* val_read;
    // std::vector<bool>* bool_read;

    if (i == 0) {
      file_path = acc_file_path;
      timestamp = &acc_val_time_;
      val_read  = &acc_val_read_;

      // bool_read = &acc_val_operational_;
    } else if (i == 1) {
      file_path = dec_file_path;
      timestamp = &dec_val_time_;
      val_read  = &dec_val_read_;

      // bool_read = &dec_val_operational_;
    } else if (i == 2) {
      file_path = em_file_path;
      timestamp = &em_val_time_;
      val_read  = &em_val_read_;

      // bool_read = &em_val_operational_;
    }

    std::ifstream file;
    std::string line;

    NavigationVector value;
  
    //int counter = 0;
    uint32_t temp_time;

    file.open(file_path);
    if (!file.is_open()) {
      log_.ERR("Fake-IMU", "Wrong file path for argument: %d", i);
    } else {
      //std::cout << "This shit works!! \n";
    while (getline(file, line)) {
      //std::cout << line << "\n";
      std::stringstream input(line);
      
      input >> temp_time;
      timestamp->push_back(temp_time);

      input >> value[0];
      value[1] = 0.0;
      value[2] = 9.8;

      log_.INFO("ADDNOISETODATA", "THING: %f", addNoiseToData(value, noise_)[1]);

      val_read->push_back(addNoiseToData(value, noise_));
      
    }
    }

    file.close();
}}



bool FakeImu::accCheckTime()
{
  uint64_t now = utils::Timer::getTimeMicros();

  uint64_t time_span = (now - ref_time_) / 1000;

  if (time_span < kAccTimeInterval*acc_count_) {
    return false;

  }
  acc_count_ = time_span/kAccTimeInterval + 1;
  return true;

}}}

  // namespace hyped::sensors
