
rock_find_qt5(Core)

rock_vizkit_plugin(motion_planning_libraries-viz
    MOC 
        PluginLoader.cpp
        MotionPlanningLibrariesStateVisualization.cpp
        MotionPlanningLibrariesSbplMprimsVisualization.cpp
        MotionPlanningLibrariesSbplSplineVisualization.cpp
    HEADERS 
        MotionPlanningLibrariesStateVisualization.hpp 
        MotionPlanningLibrariesSbplMprimsVisualization.hpp
        MotionPlanningLibrariesSbplSplineVisualization.hpp
    DEPS motion_planning_libraries
)
    
