
#include "utils/concurrent/thread.hpp"
#include "utils/io/gpio.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "utils/timer.hpp"

using hyped::utils::io::GPIO;
using hyped::utils::Logger;
using hyped::utils::Timer;

int main(int argc, char* argv[]) {
  hyped::utils::System::parseArgs(argc, argv);
  Logger log(true, 0);
  GPIO pin(66, hyped::utils::io::gpio::kOut);
  int count = 0;

  log.INFO("GPIOTEST", "STARTING");

  uint64_t start_time = Timer::getTimeMicros();
  log.INFO("GPIOTEST", "start time %u", start_time);
  uint64_t current_time = start_time;
  while (count <= 10000) {
    current_time = Timer::getTimeMicros();
    if (current_time - start_time > (60)) {     // 20000 rpm at 50 pulses per revolution in micros
      pin.set();
      pin.clear();
      start_time = current_time;
      count++;
      if (count == 1000)
        log.INFO("GPIOTEST", "Reached %d", count);
    }
  }
  log.INFO("GPIOTEST", "ENDING");
  
} // end of main
