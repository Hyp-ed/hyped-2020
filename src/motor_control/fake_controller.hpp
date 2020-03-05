/*
 * Author: Stella Antonogiannaki
 * Organisation: HYPED
 * Date: 27/01/2020
 * Description: Class for simulated Controllers
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

#ifndef MOTOR_CONTROL_FAKE_CONTROLLER_HPP_
#define MOTOR_CONTROL_FAKE_CONTROLLER_HPP_

#include "utils/logger.hpp"
#include "motor_control/controller_interface.hpp"
#include "utils/interface_factory.hpp"
#include "utils/system.hpp"

namespace hyped {

namespace motor_control {

class FakeController: public ControllerInterface {
 public:
  /**
   * @brief  Construct a new FakeController object
   */
  FakeController();
  /**
   * @brief  Initialises private fields in the controller
   */
  void initController(uint8_t id) override;
  /**
   * @brief  Applies Configuration settings
   */
  void configure() override;
  /**
   * @brief  Enter preoperational state
   */
  void enterPreOperational() override;
  /**
   * @brief  Enter operational state
   */
  void enterOperational() override;
  /**
   * @brief  Check controller state
   */
  void checkState() override;
  /**
   * @brief  Sets actual current = target current
   * @param[in] target current
   */
  void sendTargetCurrent(int32_t target_current) override;
  /**
   * @brief  Sets actual frequency = target frequency
   * @param[in] target frequency
   */
  void sendTargetFrequency(int32_t target_frequency) override;
  /**
   * @brief  Sets controller to quickstop mode
   */
  void quickStop() override;
  /**
   * @brief
   */
  void healthCheck() override;
  /**
   * @return  int32_t - actual current of the motor
   */
  int32_t getCurrent() override;
  /**
   * @return  int32_t - actual frequency of the motor
   */
  int32_t getFrequency() override;
  /**
   * @brief  Get and return the state of the controller
   * @return  state_
   */
  ControllerState getControllerState() override;
  /**
   * @brief  get and return the node id of the controller
   * @return  uint8_t - id_
   */
  uint8_t getNodeID() override;
  /**
   * @return the failure flag, critical_failure_
   */
  bool getFailure() override;
  /**
   * @brief  Get and return the temperature of the motor
   */
  uint8_t getMotorTemp() override;
  /**
   * @brief  Get and return the tempretature of the controller
   */
  uint8_t getControllerTemp() override;

 private:
  /**
   * @brief  Registers controller to recieve and transmit CAN messages
   */
  void registerController() override;
  /**
   * @brief Send a request to the motor controller to get the temperature of the motor
   */
  void updateMotorTemp() override;
  /**
   * @brief Send a request to the motor controller to get the actual frequency
   */
  void updateActualFrequency() override;
  /**
   * @brief  Send a request to the motor controller to get the actual current
   */
  void updateActualCurrent() override;
  /**
   * @brief Send a request to the motor controller to get the actual temperature of the controller
   */
  void updateControllerTemp() override;
  /**
   * @brief set critical failure flag
   */
  void throwCriticalFailure() override;

  Logger&          log_;
  atomic<int32_t>  actual_current_;
  atomic<int32_t>  actual_frequency_;
  uint8_t          id_;
  ControllerState  state_;
  bool             critical_failure_;
  uint8_t          motor_temp_;
  uint8_t          controller_temp_;
  bool             isFaulty_;
};

}
}
#endif  // MOTOR_CONTROL_FAKE_CONTROLLER_HPP_
