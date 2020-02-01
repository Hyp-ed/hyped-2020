/*
 * Authors: Martin Kristien
 * Organisation: HYPED
 * Date: January 2020
 *
 *    Copyright 2020 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licen ses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include <cstdio>
#include <cstdint>

#include "utils/concurrent/thread.hpp"
#include "utils/concurrent/lock.hpp"
#include "utils/system.hpp"

using hyped::utils::concurrent::Thread;
using hyped::utils::concurrent::Lock;
using hyped::utils::Logger;
using hyped::utils::System;

// feel free to change these
#define ITERATIONS  100000
#define STACK_SIZE  1000

/**
 * TODO: Implement push() and pop() of standard stack data structure to provide safety
 * in a multi-threaded environment. This means the data sturcture should safely operate even
 * if multiple calls to push/pop are performed in parallel.
 *
 * Hint: think where is the data-race, what is the critical section
 */
class Stack {
 public:
  Stack(Lock& lock): capacity_(STACK_SIZE), top_(0), values_{0}, lock_(lock) {}

  /**
   * @brief Add a new element onto the stack
   *
   * @param value
   * @return true iff value has been pushed onto the stack (might be full)
   */
  bool push(int value) {
    // IMPLEMENT HERE
    return false;
  }

  /**
   * @brief Remove an element from the top of the stack.
   * The storage backing the original value should be reset to 0
   * so that the value cannot be recovered, for security reasons.
   *
   * @param value output pointer where the stack value should be loaded
   * @return true iff a value has been poped (might be empty)
   */
  bool pop(int* value) {
    // IMPLEMENT HERE
    return false;
  }

  const int capacity_;              // maximum size
  int       top_;                   // current size
  int       values_[STACK_SIZE+1];  // backing storage

  Lock&     lock_;                  // use this lock for mutual exclusion
};

//--------------------------------------------------------------------------------------------------
// Driver code - do not implement, change permitted for debugging only
//--------------------------------------------------------------------------------------------------

Lock  global_lock;
Stack stack(global_lock);

class Pusher: public Thread {
  void run() override {
    for (int i = 0; i < ITERATIONS; i++) {
      while (!stack.push(42));
    }
  }
};

class Popper: public Thread {
  void run() override {
    for (int i = 0; i < ITERATIONS; i++) {
      int  value = 1;
      while (!stack.pop(&value));

      if (value != 42) {
        printf("invalid pop value of %d\n", value);
      }
    }
  }
};

int main(int argc, char* argv[]){
  System::parseArgs(argc, argv);

  printf("Running concurrent stack simulation\n");

  // create two pushers and two poppers
  Pusher pusher1, pusher2;
  Popper popper1, popper2;

  // start all threads
  pusher1.start();
  pusher2.start();
  popper1.start();
  popper2.start();

  // keep checking value at the top_ is always 0
  for (int i = 0; i < ITERATIONS; i++) {
    global_lock.lock();
    if (stack.values_[stack.top_] != 0) {
      printf("Checker error: stack is corrupted: %d %d\n", stack.values_[stack.top_], stack.top_);
    }
    global_lock.unlock();
  }

  // wait for everybody to finish
  pusher1.join();
  pusher2.join();
  popper1.join();
  popper2.join();

  printf("Concurrent stack implemented successfully\n");

  return 0;
}
