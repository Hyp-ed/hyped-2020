#include "utils/concurrent/thread.hpp"
#include "utils/io/gpio.hpp"
#include "utils/logger.hpp"
#include "utils/system.hpp"
#include "data/data.hpp"

using hyped::utils::io::GPIO;
using hyped::utils::Logger;
using hyped::utils::System;
constexpr int kMaxPulse = 8000;

int main(int argc, char* argv[]) {
  hyped::utils::System::parseArgs(argc, argv);
  Logger log = System::getLogger(); 
  GPIO gpio(67, hyped::utils::io::gpio::kIn);     // pin 67 as read
  int pin = 0;
  int count = 0;
  log.INFO("Gpio_counter_test", "Test started, initial count");
  
  while (count < kMaxPulse) {
    while (!pin) {
      gpio.read();
    }
     while (pin == 1) {
      pin = gpio.read();
    }
    count++;      // this should only increase after reading high and low pulse consecutively
    log.INFO("Gpio_counter_test", "count: %d", count);
  }
  log.INFO("Gpio_counter_test", "Final pulse count %d", count);
} // end of main