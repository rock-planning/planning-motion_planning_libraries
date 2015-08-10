set( CMAKE_BUILD_TYPE Debug )

rock_library(motion_planning_libraries
    SOURCES MotionPlanningLibraries.cpp 
        AbstractMotionPlanningLibrary.cpp
        sbpl/Sbpl.cpp 
        sbpl/SbplEnvXY.cpp
        sbpl/SbplEnvXYTHETA.cpp
        sbpl/SbplMotionPrimitives.cpp
        ompl/Ompl.cpp 
        ompl/OmplEnvXY.cpp
        ompl/OmplEnvXYTHETA.cpp
        ompl/OmplEnvARM.cpp
        ompl/OmplEnvSHERPA.cpp
        ompl/validators/TravMapValidator.cpp
        ompl/objectives/TravGridObjective.cpp
        ompl/spaces/SherpaStateSpace.cpp
    HEADERS Config.hpp 
        State.hpp
        MotionPlanningLibraries.hpp 
        AbstractMotionPlanningLibrary.hpp
        Helpers.hpp
        sbpl/Sbpl.hpp 
        sbpl/SbplEnvXY.hpp
        sbpl/SbplEnvXYTHETA.hpp
        sbpl/SbplMotionPrimitives.hpp
        ompl/Ompl.hpp 
        ompl/OmplEnvXY.hpp
        ompl/OmplEnvXYTHETA.hpp
        ompl/OmplEnvARM.hpp
        ompl/OmplEnvSHERPA.hpp
        ompl/validators/TravMapValidator.hpp 
        ompl/objectives/TravGridObjective.hpp
        ompl/spaces/SherpaStateSpace.hpp
    DEPS_PKGCONFIG envire
    DEPS_PKGCONFIG ompl
    DEPS_PKGCONFIG sbpl
    DEPS_PKGCONFIG dwa
)

rock_executable(motion_planning_libraries_bin Main.cpp
    DEPS motion_planning_libraries)
