set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

rock_testsuite(motion_planning_libraries-test suite.cpp
   test_MotionPlanning.cpp
   DEPS motion_planning_libraries
   DEPS_PKGCONFIG ompl)
