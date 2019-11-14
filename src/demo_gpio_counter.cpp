
#include "utils/concurrent/thread.hpp"
#include "utils/io/gpio.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "data/data.hpp"

using hyped::utils::io::GPIO;
using hyped::utils::Logger;

constexpr int kMaxPulse = 8000;

int main(int argc, char* argv[]) {
  hyped::utils::System::parseArgs(argc, argv);
  Logger log(true, 0);

  GPIO gpio(67, hyped::utils::io::gpio::kIn);
  int pin = 0;
  int count = 0;
  int prev = count;
  log.INFO("GPIOTEST", "Test started, initial count");
  while (count < kMaxPulse) {
    while(pin == 1){
      //log.INFO("GPIOTEST", "pin is 1");
      pin = gpio.read();

    }

    while(pin == 0){
      //log.INFO("GPIOTEST", "pin is 0");

      pin = gpio.read();

    }
    //log.INFO("GPIOTEST", "Counter %d", count);
    count++;       

  }
  log.INFO("GPIOTEST", "Final pulse count %d", count);
  
} // end of main
