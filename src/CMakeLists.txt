rock_library(global_path_planner
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(global_path_planner_bin Main.cpp
    DEPS global_path_planner)

