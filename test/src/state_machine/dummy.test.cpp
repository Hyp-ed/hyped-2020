#include "gtest/gtest.h"
#include "utils/logger.hpp"
#include "data/data.hpp"
#include "state_machine/hyped-machine.hpp"

using hyped::data::Data;

TEST(dummy_test, f)
{
  //Data &d = Data::getInstance();
  ASSERT_EQ(2,1+1);
  // ASSERT_EQ(3,1+1);
}

//     stateMachineTest

struct stateMachineTest : public ::testing::Test
{
  protected:
    hyped::utils::Logger _log;
    hyped::data::StateMachine _sm;
    Data *_d;
    void SetUp()
    {
      _d = &Data::getInstance();
      _sm = _d->getStateMachineData();
    }
};

/*
* Checks setStateMachineData correctly updates current state in hyped::data::State
*/
TEST_F(stateMachineTest, state_machine_init)
{
  _sm.current_state = hyped::data::State::kAccelerating;
  _d->setStateMachineData(_sm);
  ASSERT_EQ(_d->getStateMachineData().current_state, hyped::data::State::kAccelerating);
}

//     stateMachineMock

struct stateMachineMock : public ::testing::Test
{
    protected:
        hyped::utils::Logger _log;
        hyped::data::StateMachine _sm;
        hyped::utils::System *_sys;
        Data *_d;
            void SetUp()
            {
                char* argv[] = { "./hyped", "--fake_imu", "--fake_imu_fail", NULL};
                int argc = sizeof(argv) / sizeof(char*) - 1;
                hyped::utils::System::parseArgs(argc, argv);
              _d = &Data::getInstance();
              _sm = _d->getStateMachineData();
              _sys = &hyped::utils::System::getSystem();
            }
};

/*
* Checks that System handles fake data flags correctly
*/
TEST_F(stateMachineMock, fails_expectation)
{
   _sm.current_state = hyped::data::State::kAccelerating;
   _d->setStateMachineData(_sm);
   ASSERT_TRUE(_sys->fake_imu_fail);
}

/*
* Checks that System handles fake data failure flags correctly
*/
TEST_F(stateMachineMock, state_machine_mock)
{
  _sm.current_state = hyped::data::State::kAccelerating;
  _d->setStateMachineData(_sm);
  ASSERT_TRUE(_sys->fake_imu_fail);
}

