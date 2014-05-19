rock_library(motion_planning_libraries
    SOURCES MotionPlanningLibraries.cpp 
        AbstractMotionPlanningLibrary.cpp
        sbpl/Sbpl.cpp 
        ompl/Ompl.cpp 
        ompl/validators/TravMapValidator.cpp 
        ompl/objectives/TravGridObjective.cpp
    HEADERS Config.hpp 
        MotionPlanningLibraries.hpp 
        AbstractMotionPlanningLibrary.hpp
        sbpl/Sbpl.hpp 
        ompl/Ompl.hpp 
        ompl/validators/TravMapValidator.hpp 
        ompl/objectives/TravGridObjective.hpp
    DEPS_PKGCONFIG envire
    DEPS_PKGCONFIG ompl
    DEPS_PKGCONFIG sbpl
)

rock_executable(motion_planning_libraries_bin Main.cpp
    DEPS motion_planning_libraries)
