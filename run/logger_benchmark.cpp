/*
 * Authors : HYPED
 * Organisation: HYPED
 * Date: April 2020
 *
 *    Copyright 2020 HYPED
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *
 */

#include <thread>
#include <random>
#include <chrono>
#include "utils/logger.hpp"
#include "utils/system.hpp"

using hyped::utils::Logger;
using hyped::utils::System;

void log_stuff(Logger logger, int thread_number, int iterations);

int main(int argc, char* argv[])
{
    System::parseArgs(argc, argv);
    Logger& system_logger = hyped::utils::System::getLogger();

    Logger thread_1_logger(true, 3);
    Logger thread_2_logger(true, 3);
    Logger thread_3_logger(true, 3);

    int iterations = 1000000;

    system_logger.INFO("LOGGER BENCHMARK", "Starting benchmarks");

    auto start = std::chrono::steady_clock::now();

    std::thread thread_1(log_stuff, thread_1_logger, 1, iterations);
    std::thread thread_2(log_stuff, thread_2_logger, 2, iterations);
    std::thread thread_3(log_stuff, thread_3_logger, 3, iterations);

    thread_1.join();
    thread_2.join();
    thread_3.join();

    auto end = std::chrono::steady_clock::now();

    int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    float throughput = static_cast<float>(iterations * 3) / time_elapsed * 1000;

    system_logger.INFO("LOGGER BENCHMARK", "Time Elapsed: %d ms \t %1.3f/sec", time_elapsed, throughput);

    return 0;
}

void log_stuff(Logger logger, int thread_number, int iterations) {
    std::mt19937 engine(std::random_device{}());
    std::uniform_int_distribution<int> uniform_dist(1, 100);

    for (int i = 0; i < iterations; i++) {
        logger.INFO("LOGGER BENCHMARK", "Thread %d Iteration %d: random value %d", \
                    thread_number, i, uniform_dist(engine));
    }
}
