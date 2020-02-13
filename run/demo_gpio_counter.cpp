#include "utils/concurrent/thread.hpp"
#include "utils/io/gpio.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/timer.hpp"
#include "data/data.hpp"

using hyped::utils::concurrent::BusyThread;
using hyped::utils::concurrent::Thread;
using hyped::utils::io::GPIO;
using hyped::utils::Logger;
using hyped::utils::System;
using hyped::utils::Timer;

constexpr int kMaxTime = 4;

int main(int argc, char* argv[]) {
  hyped::utils::System::parseArgs(argc, argv);
  Logger log = System::getLogger();

  BusyThread* worker = new BusyThread();
  worker->start();

  GPIO gpio(67, hyped::utils::io::gpio::kIn);     // pin 67 as read
  int pin = 0;
  int count = 0;
  log.INFO("Gpio_counter_test", "Test started, initial count");

  uint64_t t0 = Timer::getTimeMicros();
  while (Timer::getTimeMicros() - t0 < kMaxTime*1000000) {
    while (!pin) {
      gpio.read();
    }
     while (pin == 1) {
      pin = gpio.read();
    }if (count%10000 == 0)
      log.INFO("Gpio_counter_test", "count: %5dk, time: %fs", count/1000,
                                                              (Timer::getTimeMicros() - t0)/1e6);
    count++;  // this should only increase after reading high and low pulse consecutively
  }

  log.INFO("Gpio_counter_test", "Stopping the worker");
  // TODO(Brano): Stop BusyThread via System::running_
  delete worker;

  log.INFO("Gpio_counter_test", "Final pulse count %d", count);
} // end of main
