#include "gmock/gmock.h"
#include "state_machine/hyped-machine.hpp"

class MockHypedMachine : public hyped::state_machine::HypedMachine
{
  public:
      MOCK_METHOD((void), transition, (hyped::data::State));
};
