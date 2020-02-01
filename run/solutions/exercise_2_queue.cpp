
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
#include <cstdlib>

#include "utils/concurrent/thread.hpp"
#include "utils/concurrent/lock.hpp"
#include "utils/system.hpp"

using hyped::utils::concurrent::Thread;
using hyped::utils::concurrent::Lock;
using hyped::utils::Logger;
using hyped::utils::System;

// feel free to change these
#define ITERATIONS  1000000
#define QUEUE_SIZE  1000

/**
 * TODO: Implement enqueue() and dequeue() of standard circular queue data structure to provide
 * safety in a multi-threaded environment. This means the data sturcture should safely operate even
 * if multiple calls to enqueue/dequeue are performed in parallel.
 *
 * Hint: think where is the data-race, what is the critical section
 */
class Queue {
 public:

  /**
   * @brief Add a new element at the front of the queue
   *
   * @param value
   * @return true iff value has been enqueued into the queue (might be full)
   */
  bool enqueue(int value) {
    // IMPLEMENT HERE
    // return false;

    hyped::utils::concurrent::ScopedLock L(&l);
    if (full) return false;

    values_[front_] = value;
    front_ = (front_ + 1) % QUEUE_SIZE;
    full = front_ == back_;

    return true;
  }

  /**
   * @brief Remove an element from the end of the queue.
   *
   * @param value output pointer where the queue value should be loaded
   * @return true iff a value has been dequeued (might be empty)
   */
  bool dequeued(int* value) {
    // IMPLEMENT HERE
    // return false;

    hyped::utils::concurrent::ScopedLock L(&l);
    // check if empty
    if (front_==back_ && !full) return false;

    *value = values_[back_];
    back_ = (back_ + 1) % QUEUE_SIZE;
    full = false;

    return true;
  }

  // IMPLEMENT HERE: add your own member variables
  Lock l;
  int values_[QUEUE_SIZE];
  int front_  = 0;
  int back_   = 0;
  bool full   = false;
};

//--------------------------------------------------------------------------------------------------
// Driver code - do not implement, change permitted for debugging only
//--------------------------------------------------------------------------------------------------

Queue queue;

class Enqueuer: public Thread {
 public:
  /**
   * @brief Generate random number as add them to the queue. Keep sum of enqueued numbers
   */
  void run() override {
    for (int i = 0; i < ITERATIONS; i++) {
      int value  = rand();
      total_     += value;

      while (!queue.enqueue(value));
    }
  }

  int total_ = 0;
};

class Dequeuer: public Thread {
 public:
  /**
   * @brief Sum numbers dequeued from the queue.
   */
  void run() override {
    for (int i = 0; i < ITERATIONS; i++) {
      int  value;
      while (!queue.dequeued(&value));

      total_ += value;
    }
  }

  int total_ = 0;
};

int main(int argc, char* argv[]){
  System::parseArgs(argc, argv);

  int num_threads = 1;
  if (argc == 2) {
    num_threads = std::atoi(argv[1]);
  }

  printf("Running concurrent queue simulation with %d enqueuers and %d dequeuers\n",
          num_threads, num_threads);

  // create two pushers and two poppers
  Enqueuer* enqueuers = static_cast<Enqueuer*>(malloc(num_threads*sizeof(Enqueuer)));
  Dequeuer* dequeuers = static_cast<Dequeuer*>(malloc(num_threads*sizeof(Dequeuer)));
  for (int i = 0; i < num_threads; i++) {
    new(&enqueuers[i]) Enqueuer();
    new(&dequeuers[i]) Dequeuer();
  }


  // start all threads
  for (int i = 0; i < num_threads; i++) {
    enqueuers[i].start();
    dequeuers[i].start();
  }

  // wait for everybody to finish
  for (int i = 0; i < num_threads; i++) {
    enqueuers[i].join();
    dequeuers[i].join();
  }

  // check what was enqueued has been dequeued
  int sum_enqueue = 0;
  int sum_dequeue = 0;
  for (int i = 0; i < num_threads; i++) {
    sum_enqueue += enqueuers[i].total_;
    sum_dequeue += dequeuers[i].total_;
  }

  if (sum_enqueue == sum_dequeue) {
    printf("Concurrent queue implemented successfully, %d\n", sum_enqueue);
  } else {
    printf("Concurrent queue corrupted, %d vs %d\n", sum_enqueue, sum_dequeue);
  }

  return 0;
}
