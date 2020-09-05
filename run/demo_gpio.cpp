#include <chrono>
#include <thread>

#include "utils/concurrent/thread.hpp"
#include "utils/io/gpio.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/timer.hpp"

using hyped::utils::io::GPIO;
using hyped::utils::concurrent::BusyThread;
using hyped::utils::Logger;
using hyped::utils::Timer;

// Set number of pulses to 1-20M for fast pulses and 10-100k for timed ones
constexpr int kNumPulses = 1*60*1000;  // Multiplication just to count zeros more easily

void timed_pulses(GPIO& pin, int delay_us, uint64_t t0, Logger& log) {
  uint64_t t;
  for (int i = 0; i < kNumPulses;) {
    t = Timer::getTimeMicros();
    if (t - t0 >= 60) {
      pin.set();
      pin.clear();
      t0 = t;
      ++i;
      // if (i % 10000 == 0)
      //   log.INFO("GPIOTEST", "Reached %d", i);
    }
  }
}

void fast_pulses(GPIO& pin) {
  for (int i = 0; i < kNumPulses; ++i) {
    pin.set();
    pin.clear();
  }
}

int main(int argc, char* argv[]) {
  hyped::utils::System::parseArgs(argc, argv);
  Logger log(true, 0);
  GPIO pin(66, hyped::utils::io::gpio::kOut);
  GPIO dbg_pin(67, hyped::utils::io::gpio::kOut);
  pin.clear();
  dbg_pin.clear();

  log.INFO("GPIOTEST", "STARTING");

  uint64_t start_time = Timer::getTimeMicros();
  log.INFO("GPIOTEST", "start time %.3fms", start_time/1000.0);
  uint64_t current_time = start_time;
  dbg_pin.set();

  // Comment/uncomment based on which test you want to run
  timed_pulses(pin, 60, current_time, log);  // 20000 rpm at 50 pulses per revolution ==> 60us
  // fast_pulses(pin);

  dbg_pin.clear();
  current_time = Timer::getTimeMicros();
  log.INFO("GPIOTEST", "end time: %.3fms, duration: %.3fms",
           current_time/1000.0, (current_time - start_time)/1000.0);
  std::this_thread::sleep_for(std::chrono::seconds(4));
  log.INFO("GPIOTEST", "ENDING");

} // end of main
