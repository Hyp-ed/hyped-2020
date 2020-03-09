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

#include "motor_control/controller_manager.hpp"

namespace hyped {

namespace motor_control {

ControllerManager::ControllerManager()
  : log_(System::getLogger()),
    sys_(System::getSystem()),
    data_(Data::getInstance()),
    motor_data_(data_.getMotorData()),
    motor_amount_(sys_.config->motor_control.numControllers),
    initialised(false),
    criticalError(false)
{
  // TODO(iain): create instances of controllers
}

void ControllerManager::initMotors()
{}

void ControllerManager::enterPreOperational()
{}

void ControllerManager::quickStopAll()
{}

void ControllerManager::healthCheck()
{}

bool ControllerManager::getFailure()
{
  // Stub
  return false;
}

void ControllerManager::accelerate()
{}

bool ControllerManager::isInitialised()
{
  // Stub
  return true;
}

bool ControllerManager::isCriticalFailure()
{
  // Stub
  return false;
}


void ControllerManager::registerControllers()
{}

void ControllerManager::configureControllers()
{}

void ControllerManager::prepareMotors()
{}

}}  // hyped::motor_control
