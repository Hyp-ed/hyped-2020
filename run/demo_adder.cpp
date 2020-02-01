/*
 * Authors: Kornelija Sukyte and Martin Kristien
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

#define ITERATIONS 10000000

class Increment : public Thread {
 public:
  Increment(uint64_t& counter_ptr): value_(counter_ptr) {}

  void run() override {
    for(uint64_t i = 0; i < ITERATIONS; i++){
      ++value_;
    }
  }

  uint64_t& value_;
};

int main(int argc, char* argv[]){
  System::parseArgs(argc, argv);
  uint64_t number = 0;

  Increment* increment_objects;

  int num_threads = 1;
  if (argc == 2) {
    num_threads = std::atoi(argv[1]);
  }

  printf("using %d threads\n", num_threads);

  increment_objects = static_cast<Increment*>(malloc(num_threads*sizeof(Increment)));
  for (int i = 0; i < num_threads; i++) {
    new(&increment_objects[i]) Increment(number);
    increment_objects[i].start();
  }

  for (int i = 0; i < num_threads; i++) {
    increment_objects[i].join();
  }

  printf("expected %ld vs %ld actual\n", num_threads*ITERATIONS, number);
  return 0;
}
