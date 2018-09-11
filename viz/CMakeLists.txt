set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

rock_vizkit_plugin(motion_planning_libraries-viz
    PluginLoader.cpp
    MOC 
        MotionPlanningLibrariesStateVisualization.cpp 
        MotionPlanningLibrariesSbplMprimsVisualization.cpp
    HEADERS 
        MotionPlanningLibrariesStateVisualization.hpp 
        MotionPlanningLibrariesSbplMprimsVisualization.hpp
    DEPS motion_planning_libraries
)
    
