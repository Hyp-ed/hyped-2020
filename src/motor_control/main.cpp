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

#include "motor_control/main.hpp"

namespace hyped {

namespace motor_control {

Main::Main(uint8_t id, utils::Logger& log)
  : Thread(id, log),
    log_(log),
    sys_(System::getSystem()),
    data_(Data::getInstance()),
    is_running_(true)
{
  current_state_ = data_.getStateMachineData().current_state;
  previous_state_ = State::kInvalid;
  controller_manager_ = new ControllerManager();
}

void Main::run()
{
log_.INFO("Motor", "Thread started");

  Motors motor_data = data_.getMotorData();

  while (is_running_ && sys_.running_) {
    // Get the current state of the system from the state machine's data
    current_state_ = data_.getStateMachineData().current_state;

    if (current_state_ == State::kIdle) {  // Initialize motors
      if (current_state_ != previous_state_) {
        log_.INFO("Motor", "State idle");
        previous_state_ = current_state_;
      }
      if (motor_data.module_status != ModuleStatus::kInit) {
        motor_data.module_status = ModuleStatus::kInit;
        data_.setMotorData(motor_data);
      }
    } else if (current_state_ == State::kCalibrating) {
        // Calculate slip values
        if (previous_state_ != current_state_) {
          log_.INFO("Motor", "State Calibrating");
          previous_state_ = current_state_;
        }
        if (!controller_manager_->isInitialised()) {
          controller_manager_->initMotors();
          if (controller_manager_->isCriticalFailure()) {
            motor_data.module_status = ModuleStatus::kCriticalFailure;
            data_.setMotorData(motor_data);
            is_running_ = false;
          }
        }
        if (controller_manager_->isInitialised() &&
            motor_data.module_status != ModuleStatus::kReady) {
          motor_data.module_status = ModuleStatus::kReady;
          data_.setMotorData(motor_data);
        }
    } else if (current_state_ == State::kReady) {
          // Standby and wait
          if (previous_state_ != current_state_) {
            log_.INFO("Motor", "State Ready");
            previous_state_ = current_state_;
            controller_manager_->sendOperationalCommand();
          }
    } else if (current_state_ == State::kAccelerating) {
          // Accelerate the motors
          if (previous_state_ != current_state_) {
            log_.INFO("Motor", "State Accelerating");
            previous_state_ = current_state_;
          }
          controller_manager_->accelerate();
    } else if (current_state_ == State::kNominalBraking) {
          // Stop all motors
          if (previous_state_ != current_state_) {
            log_.INFO("Motor", "State NominalBraking");
            previous_state_ = current_state_;
          }
          controller_manager_->quickStopAll();
    } else if (current_state_ == State::kEmergencyBraking) {
          // Stop all motors
          if (previous_state_ != current_state_) {
            log_.INFO("Motor", "State EmergencyBraking");
            previous_state_ = current_state_;
          }
          controller_manager_->quickStopAll();
    } else if (current_state_ == State::kExiting) {
          // Leave motors off while exiting
          if (previous_state_ != current_state_) {
            log_.INFO("Motor", "State Exiting");
            previous_state_ = current_state_;
          }
          controller_manager_->enterPreOperational();
    } else if (current_state_ == State::kFailureStopped) {
          // Enter preoperational
          if (previous_state_ != current_state_) {
            log_.INFO("Motor", "State FailureStopped");
            previous_state_ = current_state_;
          }
          controller_manager_->enterPreOperational();
    } else if (current_state_ == State::kFinished) {
          // Turn off Motors
          if (previous_state_ != current_state_) {
            log_.INFO("Motor", "State Finished");
            previous_state_ = current_state_;
          }
          controller_manager_->enterPreOperational();
    } else if (current_state_ == State::kRunComplete) {
          // Run complete
          if (previous_state_ != current_state_) {
            log_.INFO("Motor", "State RunComplete");
            previous_state_ = current_state_;
          }
          controller_manager_->quickStopAll();
    } else {
          // Unknown State
          if (previous_state_ != current_state_) {
            log_.INFO("Motor", "State Unknown");
            previous_state_ = current_state_;
          }
          is_running_ = false;
          log_.ERR("Motor", "Unknown state");
          motor_data.module_status = ModuleStatus::kCriticalFailure;
          data_.setMotorData(motor_data);
          controller_manager_->quickStopAll();
    }
    sleep(1);
  }

  log_.INFO("Motor", "Thread shutting down");
}

}  // namespace hyped
}  // namespace motor_control
