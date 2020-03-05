#include "utils/logger.hpp"
#include "utils/config.hpp"
#include "utils/system.hpp"
#include "motor_control/controller_interface.hpp"

using namespace hyped;

int main(int argc, char* argv[])
{
  utils::System::parseArgs(argc, argv);
  utils::System& sys = utils::System::getSystem();
  utils::Logger& log = utils::System::getLogger();

  motor_control::ControllerInterface* controller = 
    sys.config->interfaceFactory.getControllerInterfaceInstance();

  controller->initController(1);
  controller->configure();
  controller->enterOperational();
  controller->sendTargetCurrent(100000);
  controller->sendTargetFrequency(100000);
  controller->quickStop();
  int temp = controller->getMotorTemp();
  log.INFO("TEST", "Controller %d: %d C", 1, temp);  
  return 0;
}
