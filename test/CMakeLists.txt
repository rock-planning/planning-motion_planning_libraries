rock_testsuite(motion_planning_libraries-test suite.cpp
   test_MotionPlanning.cpp
   test_SbplSplineMotionPrimitives.cpp
   DEPS_PKGCONFIG motion_planning_libraries
   DEPS_PKGCONFIG ompl)
