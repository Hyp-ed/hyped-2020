/*
 * Authors: Branislav Pilnan
 * Organisation: HYPED
 * Date: February 2020
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
#include <vector>

#include "utils/concurrent/thread.hpp"
#include "utils/concurrent/lock.hpp"
#include "utils/timer.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"

using hyped::utils::concurrent::Thread;
using hyped::utils::concurrent::Lock;
using hyped::utils::concurrent::ScopedLock;
using hyped::utils::Timer;
using hyped::utils::Logger;
using hyped::utils::System;

#define ROWS 1000
#define COLS 1000000

class Sum : public Thread {
 public:
  Sum(std::vector<std::vector<double> >& matrix, unsigned int& row_counter, Lock& l):
    matrix_(matrix), row_counter_(row_counter), lck_(l)
  {}

  void run() override
  {
    while (row_counter_ < matrix_.size()) {
      lck_.lock();
      int current_row = row_counter_;
      ++row_counter_;
      lck_.unlock();

      // Add all the elements in current row to the total sum
      for (auto &&x : matrix_[current_row]) {
        sum_ += x;
      }
    }
  }

  double get_sum()
  {
    return sum_;
  }

 private:
  std::vector<std::vector<double> >& matrix_;
  unsigned int& row_counter_;
  Lock& lck_;
  double sum_ = 0;
};

int main(int argc, char* argv[])
{
  System::parseArgs(argc, argv);
  std::vector<std::vector<double> > matrix(ROWS, std::vector<double>(COLS, 1.0));
  unsigned int row_counter = 0;
  Lock lck;

  Sum* sum_objects;

  int num_threads = 1;
  if (argc == 2) {
    num_threads = std::atoi(argv[1]);
  }

  printf("using %d threads\n", num_threads);
  Timer timer;
  timer.start();

  sum_objects = static_cast<Sum*>(malloc(num_threads*sizeof(Sum)));
  for (int i = 0; i < num_threads; i++) {
    new(&sum_objects[i]) Sum(matrix, row_counter, lck);
    sum_objects[i].start();
  }

  double total_sum = 0;
  for (int i = 0; i < num_threads; i++) {
    sum_objects[i].join();
    total_sum += sum_objects[i].get_sum();
    printf("Thread%d sum: %.0f\n", i, sum_objects[i].get_sum());
  }
  timer.stop();

  printf("Time: %fs\n", timer.getSeconds());
  printf("expected %ld vs %.0f actual\n", ROWS*COLS, total_sum);
  return 0;
}