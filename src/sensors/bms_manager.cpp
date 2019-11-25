/*
 * Author: Gregory Dayao and Jack Horsburgh
 * Organisation: HYPED
 * Date: 20/06/18
 * Description:
 * BMS manager for getting battery data and pushes to data struct.
 * Checks whether batteries are in range and enters emergency state if fails.
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


#include "sensors/bms_manager.hpp"

#include "sensors/bms.hpp"
#include "utils/timer.hpp"
#include "sensors/fake_batteries.hpp"
#include "utils/config.hpp"

namespace hyped {
namespace sensors {

BmsManager::BmsManager(Logger& log)
    : ManagerInterface(log),
      sys_(utils::System::getSystem()),
      data_(Data::getInstance())
{
  old_timestamp_ = utils::Timer::getTimeMicros();
  if (!(sys_.fake_batteries || sys_.fake_batteries_fail)) {
    // create BMS LP
    for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
      BMS* bms = new BMS(i, log_);
      bms->start();
      bms_[i] = bms;
    }
    // fake HP for state machine tests
    if (!sys_.fake_highpower) {
      // create BMS HP
      for (int i = 0; i < data::Batteries::kNumHPBatteries; i++) {
        bms_[i + data::Batteries::kNumLPBatteries] = new BMSHP(i, log_);
      }
    } else {
      // fake HP battery only
      for (int i = 0; i < data::Batteries::kNumHPBatteries; i++) {
        bms_[i + data::Batteries::kNumLPBatteries] = new FakeBatteries(log_, false, false);
      }
    }

    if (!sys_.battery_test) {
      // Set SSR switches for real system

      // IMD ssr
      imd_out_ = new GPIO(sys_.config->sensors.IMDOut, utils::io::gpio::kOut);
      imd_out_->set();
      log_.INFO("BMS-MANAGER", "IMD has been initialised SET");

      // clear HPSSRs if default is high
      for (int i = 0; i < data::Batteries::kNumHPBatteries; i++) {
        hp_ssr_[i] = new GPIO(sys_.config->sensors.HPSSR[i], utils::io::gpio::kOut);
        hp_ssr_[i]->clear();      // HP off until kReady State
        log_.INFO("BMS-MANAGER", "HP SSR %d has been initialised CLEAR", i);
      }
      hp_master_ = new GPIO(sys_.config->sensors.hp_master, utils::io::gpio::kOut);
      hp_master_->clear();
      log_.INFO("BMS-MANAGER", "HP SSRs has been initialised CLEAR");

      // Set embrakes ssr
      embrakes_ssr_ = new GPIO(sys_.config->sensors.embrakes, utils::io::gpio::kOut);
      embrakes_ssr_->set();
      log_.INFO("BMS-MANAGER", "Embrake SSR has been set");
    }
  } else if (sys_.fake_batteries_fail) {
    // fake batteries fail here
    for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
      bms_[i] = new FakeBatteries(log_, true, true);
    }
    for (int i = 0; i < data::Batteries::kNumHPBatteries; i++) {
      bms_[i + data::Batteries::kNumLPBatteries] = new FakeBatteries(log_, false, true);
    }
  } else {
    // fake batteries here
    for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
      bms_[i] = new FakeBatteries(log_, true, false);
    }
    for (int i = 0; i < data::Batteries::kNumHPBatteries; i++) {
      bms_[i + data::Batteries::kNumLPBatteries] = new FakeBatteries(log_, false, false);
    }
  }

  previous_state_ = data_.getStateMachineData().current_state;
  // kInit for SM transition
  batteries_ = data_.getBatteriesData();
  batteries_.module_status = data::ModuleStatus::kInit;
  data_.setBatteriesData(batteries_);
  Thread::yield();
  start_time_ = utils::Timer::getTimeMicros();
  log_.INFO("BMS-MANAGER", "batteries data has been initialised");
}


void BmsManager::run()
{
  while (sys_.running_) {
    // keep updating data_ based on values read from sensors
    for (int i = 0; i < data::Batteries::kNumLPBatteries; i++) {
      bms_[i]->getData(&batteries_.low_power_batteries[i]);
      if (!bms_[i]->isOnline())
        batteries_.low_power_batteries[i].voltage = 0;
    }
    for (int i = 0; i < data::Batteries::kNumHPBatteries; i++) {
      bms_[i + data::Batteries::kNumLPBatteries]->getData(&batteries_.high_power_batteries[i]);
      if (!bms_[i + data::Batteries::kNumLPBatteries]->isOnline())
        batteries_.high_power_batteries[i].voltage = 0;
    }

    // publish the new data
    data_.setBatteriesData(batteries_);
  }
}

bool BmsManager::batteriesInRange()
{
}

}}  // namespace hyped::sensors
