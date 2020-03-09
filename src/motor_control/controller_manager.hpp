/*
* Author: Iain Macpherson
* Organisation: HYPED
* Date: 2020
* Description: Handles the different states of the state machine
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

#ifndef MOTOR_CONTROL_CONTROLLER_MANAGER_HPP_
#define MOTOR_CONTROL_CONTROLLER_MANAGER_HPP_

#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/config.hpp"
#include "data/data.hpp"
#include "motor_control/controller_interface.hpp"

namespace hyped {

namespace motor_control {

using utils::Logger;
using utils::System;
using utils::Config;
using data::Data;
using data::Motors;
using motor_control::ControllerInterface;

class ControllerManager {
 public:
  /**
   * @brief {Construct the ControllerManager, will create the required number of motors}
   */
  ControllerManager();
  ~ControllerManager();
  /**
   * @brief { Sends the desired settings to the motors }
   */
  void initMotors();
  /**
   * @brief { Changes the state of the motor controller to preOperational }
   */
  void enterPreOperational();
  /**
   * @brief { Stops all motors }
   */
  void quickStopAll();

  /**
   * @brief { Checks the motor controller's health }
   */
  void healthCheck();

  /**
   * @brief { Checks if the motor controller's error registers }
   */
  bool getFailure();

  /**
   * @brief { Tells the controllers to start accelerating the motors }
   */
  void accelerate();

  /**
   * @brief { Returns if the motors are initialised already }
   */
  bool isInitialised();

  /**
     * @brief Returns if a critical error ocurred
   */
  bool isCriticalFailure();

  /**
     * @brief sends the enter operational command
   */
  void sendOperationalCommand();

 private:
  /**
   * @brief { Registers the controllers to handle CAN transmissions }
   */
  void registerControllers();

  /**
   * @brief { Configures the controllers }
   */
  void configureControllers();

  /**
   * @brief { Send settings data to the motors }
   */
  void prepareMotors();

  Logger &log_;
  System &sys_;
  Data   &data_;
  Motors motor_data_;
  int motor_amount_;
  bool initialised;
  bool criticalError;
  ControllerInterface **controllers;
};

}}  // hyped::motor_control

#endif  // MOTOR_CONTROL_CONTROLLER_MANAGER_HPP_
