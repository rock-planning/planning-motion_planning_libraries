# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.0)
project(motion_planning_libraries VERSION 0.1 DESCRIPTION "Collection of motion planning libraries")
find_package(Rock)
set(ROCK_TEST_ENABLED ON)
rock_init(motion_planning_libraries 0.1)
rock_standard_layout()
