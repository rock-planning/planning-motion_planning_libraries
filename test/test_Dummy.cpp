#include <boost/test/unit_test.hpp>
#include <global_path_planner/Dummy.hpp>

using namespace global_path_planner;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    global_path_planner::DummyClass dummy;
    dummy.welcome();
}
