rock_library(global_path_planner
    SOURCES GlobalPathPlanner.cpp validators/TravMapValidator.cpp objectives/PathClearance.cpp
    HEADERS GlobalPathPlanner.hpp validators/TravMapValidator.hpp objectives/PathClearance.hpp
    DEPS_PKGCONFIG ompl
    DEPS_PKGCONFIG envire
)

rock_executable(global_path_planner_bin Main.cpp
    DEPS global_path_planner)

