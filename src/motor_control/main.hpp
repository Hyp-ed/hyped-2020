/*
 * Author: Stella Antonogiannaki
 * Organisation: HYPED
 * Date: 25/01/2020
 * Description: Main class for motor control submodule
 *
 *  Copyright 2019 HYPED
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file
 *  except in compliance with the License. You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software distributed under
 *  the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *  either express or implied. See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef MOTOR_CONTROL_MAIN_HPP_
#define MOTOR_CONTROL_MAIN_HPP_

#include "utils/concurrent/thread.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "data/data.hpp"
#include "motor_control/controller_manager.hpp"


namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;
using utils::System;
using data::Data;
using data::State;
using data::ModuleStatus;
using data::Motors;
using motor_control::ControllerManager;


namespace motor_control {

class Main : public Thread {
 public:
  /**
   * @brief {Construct the motor control main thread}
   */
  Main(uint8_t id, utils::Logger& log);
  /**
   * @brief {Entry point for the motor control thread, responds to state machine transitions}
   */
  void run() override;

 private:
  Logger&  log_;
  System& sys_;
  Data& data_;
  ControllerManager* controller_manager_;
  State current_state_;
  State previous_state_;
  bool is_running_;
};

}  // namespace motor_control
}  // namespace hyped

#endif  // MOTOR_CONTROL_MAIN_HPP_
