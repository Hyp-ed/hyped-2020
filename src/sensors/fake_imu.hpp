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
#ifndef SENSORS_FAKE_IMU_HPP_
#define SENSORS_FAKE_IMU_HPP_

#include <string>
#include <vector>

#include "sensors/interface.hpp"
#include "utils/logger.hpp"
#include "data/data.hpp"

namespace hyped {

using data::ImuData;
using data::DataPoint;
using data::NavigationType;
using data::NavigationVector;
using data::State;

namespace sensors {

/*
 * @brief    This class is to imitate an IMU. This works by calling the constructor once
 *           and calling getData function multiple times at different time periods to produce
 *           reading that will be used by other classes.
 */
class FakeImu : public ImuInterface {
 public:
  FakeImu(utils::Logger& log_,
          std::string acc_file_path,
          std::string dec_file_path,
          std::string em_file_path,
          bool is_fail,
          State fail_state,
          float noise = 0.2);           // if want to change

  bool isOnline() override { return true; }

  void getData(ImuData* imu) override;



 private:
  utils::Logger&       log_;
  const uint64_t kAccTimeInterval = 50;

  /**
   * @return ImuData from file, given current state (calib, acc, dec)
   */
  ImuData getAccValue();


  /**
   * @brief needs to be added to data in getAccValue
   *
   * @param value
   * @param noise
   * @return NavigationVector
   */
  static NavigationVector addNoiseToData(NavigationVector value, float noise);

  /**
   * @brief
   *
   */
  void setFailure();


  /**
   * @brief read into vectors below
   *
   * @param acc_file_path
   * @param dec_file_path
   * @param em_file_path
   */
  void readDataFromFile(std::string acc_file_path,
                        std::string dec_file_path,
                        std::string em_file_path);

  /**
   * @brief check whether it is time to read from vector
   *
   * @return true
   * @return false
   */
  bool accCheckTime();


  ImuData acc_fail_;
  ImuData acc_zero_;
  ImuData* prev_acc_;
  std::array<NavigationVector, data::ImuData::kFifoSize> fifo_acc_;


  /**
   * @brief storing values from data files in readDataFromFile
   *
   */
  std::vector<NavigationVector> acc_val_read_;
  std::vector<uint32_t>         acc_val_time_;
  std::vector<NavigationVector> dec_val_read_;
  std::vector<uint32_t>         dec_val_time_;
  std::vector<NavigationVector> em_val_read_;
  std::vector<uint32_t>         em_val_time_;
  std::vector<NavigationVector>* read_;
  std::vector<uint32_t>*         time_;

  /**
   * @brief array index counter
   *
   */
  int64_t acc_count_;
  uint64_t ref_time_;
  bool failure_happened_;
  bool is_fail_;
  bool fail_now_;
  State fail_state_;
  uint64_t failure_time_;
  float noise_;
  data::Data&  data_;
};


}}  // namespace hyped::sensors


#endif  // SENSORS_FAKE_IMU_HPP_
