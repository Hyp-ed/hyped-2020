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

#include "fake_controller.hpp"

namespace hyped {
namespace motor_control {

void FakeController::initController()
{
}

void FakeController::configure()
{
}

void FakeController::enterPreOperational()
{
}

void FakeController::enterOperational()
{
}

void FakeController::checkState()
{
}

void FakeController::sendTargetCurrent(int32_t target_current)
{
}

void FakeController::sendTargetFrequency(int32_t target_velocity)
{
}

void FakeController::quickStop()
{
}

void FakeController::healthCheck()
{
}

int32_t FakeController::getCurrent()
{
}

int32_t FakeController::getFrequency()
{
}

ControllerState FakeController::getControllerState()
{
}

uint8_t FakeController::getNodeID()
{
}

bool FakeController::getFailure()
{
}

uint8_t FakeController::getMotorTemp()
{
}

uint8_t FakeController::getControllerTemp()
{
}

void FakeController::registerController()
{
}

void FakeController::updateMotorTemp()
{
}

void FakeController::updateControllerTemp()
{
}

void FakeController::updateActualCurrent()
{
}

void FakeController::updateActualFrequency()
{
}

void FakeController::throwCriticalFailure()
{
}

}
}
