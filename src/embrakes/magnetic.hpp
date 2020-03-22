#ifndef EMBRAKES_MAGNETIC_HPP_
#define EMBRAKES_MAGNETIC_HPP_

#include "utils/timer.hpp"
#include "utils/logger.hpp"
#include "utils/io/gpio.hpp"
#include "utils/system.hpp"
#include "utils/concurrent/thread.hpp"
#include "data/data.hpp"

namespace hyped {

using utils::concurrent::Thread;
using utils::Logger;
using utils::io::GPIO;
using data::ModuleStatus;
namespace embrakes {

class Magnetic {

  public:
    /**
     * @brief Construct a new magnetic brake object
     *
     */
    Magnetic(Logger& log);



};

}
}

#endif
