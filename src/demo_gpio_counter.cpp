
#include "utils/concurrent/thread.hpp"
#include "utils/io/gpio.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "data/data.hpp"

using hyped::utils::io::GPIO;
using hyped::utils::Logger;

constexpr int kMaxPulse = 1000;

int main(int argc, char* argv[]) {
  hyped::utils::System::parseArgs(argc, argv);
  Logger log(true, 0);

  GPIO gpio(67, hyped::utils::io::gpio::kIn);

  int count = 0;
  int prev = count;

  while (count < kMaxPulse) {
    count += gpio.read();       // read() faster than wait()
    if (count > prev) {
      // pulse read
    }  
  }
  log.INFO("GPIOTEST", "Final pulse count %d", count);
  
} // end of main
