rock_library(global_path_planner
    SOURCES GlobalPathPlanner.cpp validators/TravMapValidator.cpp validators/TurningValidator.cpp objectives/PathClearance.cpp objectives/TravGridObjective.cpp
    HEADERS GlobalPathPlanner.hpp validators/TravMapValidator.hpp validators/TurningValidator.hpp objectives/PathClearance.hpp objectives/TravGridObjective.hpp
    DEPS_PKGCONFIG ompl
    DEPS_PKGCONFIG envire
)

rock_executable(global_path_planner_bin Main.cpp
    DEPS global_path_planner)

