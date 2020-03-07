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

using hyped::utils::System;

namespace hyped {
namespace motor_control {

FakeController::FakeController()
  : log_(hyped::utils::System::getLogger()),
    actual_current_(0),
    actual_frequency_(0),
    state_(ControllerState::kNotReadyToSwitchOn),
    critical_failure_(false),
    motor_temp_(0),
    controller_temp_(0),
    timer_started_(false),
    start_time_(0)
  {}

void FakeController::initController(uint8_t id)
{
  id_ = id;
  isFaulty_ = false;
}

void FakeController::configure()
{
  log_.INFO("MOTOR", "Controller %d: Configuring", id_);
}

void FakeController::startTimer()
{
  start_time_ = utils::Timer::getTimeMicros();
  timer_started_ = true;
  fail_time_ = std::rand() % 20000000 + 1000000;
}

void FakeController::enterPreOperational()
{
  if (state_ != kSwitchOnDisabled) {
    log_.DBG1("MOTOR", "Controller %d: Shutting down motor", id_);
    state_ = kSwitchOnDisabled;
    actual_current_ = 0;
    actual_frequency_ = 0;
  }
}

void FakeController::enterOperational()
{
  state_ = kOperationEnabled;
  log_.DBG1("MOTOR", "Controller %d: Entering Operational", id_);
}

void FakeController::checkState()
{
  log_.DBG1("MOTOR", "Controller %d: Checking status", id_);
}

void FakeController::sendTargetCurrent(int32_t target_current)
{
  if (!timer_started_) {
    startTimer();
  }
  log_.DBG2("MOTOR", "Controller %d: Updating target velocity to %d", id_, target_current);
  actual_current_ = target_current;
}

void FakeController::sendTargetFrequency(int32_t target_frequency)
{
  if (!timer_started_) {
    startTimer();
  }
  log_.DBG2("MOTOR", "Controller %d: Updating target velocity to %d", id_, target_frequency);
  actual_frequency_ = target_frequency;
}

void FakeController::quickStop()
{
  log_.DBG1("MOTOR", "Controller %d: Sending quick stop command", id_);
}

void FakeController::healthCheck()
{
  if (isFaulty_) {
    if (fail_time_ <= (timer.getMicros() - start_time_)) {
      critical_failure_ = true;
    }
  }
  if (critical_failure_) {
     throwCriticalFailure();
     log_.ERR("MOTOR", "Controller %d: Fake Critical Failure", id_);
  }
}

int32_t FakeController::getCurrent()
{
  return actual_current_;
}

int32_t FakeController::getFrequency()
{
  return actual_frequency_;
}

ControllerState FakeController::getControllerState()
{
  return state_;
}

uint8_t FakeController::getNodeID()
{
  return id_;
}

bool FakeController::getFailure()
{
  return critical_failure_;
}

uint8_t FakeController::getMotorTemp()
{
  return motor_temp_;
}

uint8_t FakeController::getControllerTemp()
{
  return controller_temp_;
}

void FakeController::registerController()
{/*EMPTY*/}

void FakeController::updateMotorTemp()
{/*EMPTY*/}

void FakeController::updateControllerTemp()
{/*EMPTY*/}

void FakeController::updateActualCurrent()
{/*EMPTY*/}

void FakeController::updateActualFrequency()
{/*EMPTY*/}

void FakeController::throwCriticalFailure()
{
  critical_failure_ = true;
  log_.ERR("MOTOR", "Controller %d: Fake critical failure", id_);
}

ControllerInterface* createFakeController()
{
  return new FakeController();
}
// register the implementation with the factory
bool reg_impl = utils::InterfaceFactory<ControllerInterface>
                ::registerCreator("FakeController", createFakeController);

}
}
