rock_library(global_path_planner
    SOURCES GlobalPathPlanner.cpp sbpl/Sbpl.cpp ompl/Ompl.cpp ompl/validators/TravMapValidator.cpp ompl/objectives/TravGridObjective.cpp
    HEADERS GlobalPathPlanner.hpp sbpl/Sbpl.hpp ompl/Ompl.hpp ompl/validators/TravMapValidator.hpp ompl/objectives/TravGridObjective.hpp
    DEPS_PKGCONFIG ompl
    DEPS_PKGCONFIG envire
)

rock_executable(global_path_planner_bin Main.cpp
    DEPS global_path_planner)

