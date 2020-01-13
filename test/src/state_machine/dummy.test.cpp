#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "utils/logger.hpp"
#include "data/data.hpp"
#include "state_machine/hyped-machine.hpp"
#include "dummt.test.h"

//     stateMachineTest

struct stateMachineTest : public ::testing::Test
{
  protected:
    hyped::utils::Logger _log;
    hyped::data::StateMachine _sm;
    hyped::data::Data *_d;
    void SetUp()
    {
      _d = &hyped::data::Data::getInstance();
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
        hyped::data::Data *_d;
            void SetUp()
            {
                char* argv[] = { "./hyped", "--fake_imu", "--fake_imu_fail", NULL};
                int argc = sizeof(argv) / sizeof(char*) - 1;
                hyped::utils::System::parseArgs(argc, argv);
              _d = &hyped::data::Data::getInstance();
              _sm = _d->getStateMachineData();
              _sys = &hyped::utils::System::getSystem();
            }
};

/*
* Checks that System handles fake data flags correctly
*/
TEST_F(stateMachineMock, fails_expectation)
{
   _d->setStateMachineData(_sm);
   ASSERT_TRUE(_sys->fake_imu_fail);
}

/*
* Checks that System handles fake data failure flags correctly
*/
TEST_F(stateMachineMock, state_machine_mock)
{
  _d->setStateMachineData(_sm);
  ASSERT_TRUE(_sys->fake_imu_fail);
}

/*
 * Google mock
 */

using ::testing::AtLeast;

TEST(StateTransitionTest, AcceleratingDecelerating)
{
  hyped::state_machine::main main_stm;
  MockHypedMachine *hypedMachine;
  EXPECT_CALL( *hypedMachine, transition(hyped::data::State::kNominalBraking))
    .Times(AtLeast(1));

}





