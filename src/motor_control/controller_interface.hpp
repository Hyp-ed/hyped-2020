/*
 * Author: Stella Antonogiannaki
 * Organisation: HYPED
 * Date: 25/01/2020
 * Description: Interface for Controller class
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

#ifndef  MOTOR_CONTROL_CONTROLLER_INTERFACE_HPP_
#define  MOTOR_CONTROL_CONTROLLER_INTERFACE_HPP_

#include <atomic>

#include "utils/logger.hpp"

namespace hyped {

using std::atomic;
using utils::Logger;

namespace motor_control {

enum ControllerState {
  kNotReadyToSwitchOn,
  kSwitchOnDisabled,
  kReadyToSwitchOn,
  kSwitchedOn,
  kOperationEnabled,
  kQuickStopActive,
  kFaultReactionActive,
  kFault,
  };

class ControllerInterface {
 public:
  virtual void initController(Logger& log, uint8_t id, bool isFaulty) = 0;
  virtual void configure() = 0;
  virtual void enterOperational() = 0;
  virtual void enterPreOperational() = 0;
  virtual void checkState() = 0;
  virtual void sendTargetCurrent(int32_t target_current) = 0;
  virtual void sendTargetFrequency(int32_t target_frequency) = 0;
  virtual void quickStop() = 0;
  virtual void healthCheck() = 0;
  virtual int32_t getCurrent() = 0;
  virtual int32_t getFrequency() = 0;
  virtual ControllerState getControllerState() = 0;
  virtual uint8_t getNodeID() = 0;
  virtual bool getFailure() = 0;
  virtual uint8_t getMotorTemp() = 0;
  virtual uint8_t getControllerTemp() = 0;

 private:
  virtual void registerController() = 0;
  virtual void updateMotorTemp() = 0;
  virtual void updateActualCurrent() = 0;
  virtual void updateActualFrequency() = 0;
  virtual void updateControllerTemp() = 0;
  virtual void throwCriticalFailure() = 0;

  atomic<int32_t>  actual_current_;
  atomic<int32_t>  actual_frequency_;
  Logger&          log_;
  uint8_t          id_;
  ControllerState  state_;
  bool             critical_failure_;
  uint8_t          motor_temp_;
  uint8_t          controller_temp;
  bool             isFaulty_;
};

}  // namespace motor_control

}  // namespace hyped

#endif  // MOTOR_CONTROL_CONTROLLER_INTERFACE_HPP_
