#include <array>
#include <atomic>
#include <cstdio>

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

constexpr int kMaxTime = 32;
constexpr int kNumWorkers = 1;  // Number of busy threads

class SimpleCounter : public Thread {
 public:
  SimpleCounter();

  void run() override;
  int get_count() { return count_.load(); }
  void reset_count() { count_ = 0; }

 private:
  System&         sys_;
  std::atomic_int count_;
};

SimpleCounter::SimpleCounter() : sys_(System::getSystem()), count_(0) { /* EMPTY */ }

void SimpleCounter::run()
{
  GPIO gpio(67, hyped::utils::io::gpio::kIn);  // pin 67 as read
  int pin = 0;
  while (sys_.running_) {
    while (!pin && sys_.running_) {
      pin = gpio.read();
    }
    while (pin == 1 && sys_.running_) {
      pin = gpio.read();
    }
    ++count_;
  }
}

int main(int argc, char* argv[]) {
  hyped::utils::System::parseArgs(argc, argv);
  Logger  log = System::getLogger();
  System& sys = System::getSystem();

  std::array<BusyThread*, kNumWorkers> workers;
  for (BusyThread* worker : workers) {
    worker = new BusyThread();
    worker->start();
  }

  SimpleCounter counter;
  counter.start();

  Thread::sleep(500);
  char input;
  do {
    std::printf("Enter c to display pulse count, r to reset the count to 0, or q to quit: ");
    // Read response, ignoring newlines
     do {
      int num_read = std::scanf("%c", &input);  // Read user input
      if (num_read != 1) {
        log.ERR("Gpio_counter_test", "scanf filled %d arguments instead of 1", num_read);
        break;
      }
    } while (input == '\n' || input == '\r');

    if (input == 'c')
      std::printf("Pulse count: %d\n\n", counter.get_count());
    if (input == 'r')
      counter.reset_count();
  } while (input != 'q');

  log.INFO("Gpio_counter_test", "Stopping the counter");
  sys.running_ = false;
  counter.join();

  log.INFO("Gpio_counter_test", "Stopping the worker");
  // TODO(Brano): Stop BusyThread via System::running_
  for (BusyThread* worker : workers) {
    delete worker;
  }

  log.INFO("Gpio_counter_test", "Final pulse count %d", counter.get_count());
} // end of main
