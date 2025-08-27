
rock_find_qt4(OPTIONAL Core)
rock_find_qt5(OPTIONAL Core)

include(RockQt)

rock_qt_vizkit_plugin(
    TARGETPREFIX motion_planning_libraries-viz
    QT4_SUFFIX ""
    MISSINGQTDEPS_NOBUILD
    SOURCES
        PluginLoader.cpp
        MotionPlanningLibrariesStateVisualization.cpp
        MotionPlanningLibrariesSbplMprimsVisualization.cpp
        MotionPlanningLibrariesSbplSplineVisualization.cpp
    MOC
        PluginLoader.hpp
        MotionPlanningLibrariesStateVisualization.hpp
        MotionPlanningLibrariesSbplMprimsVisualization.hpp
        MotionPlanningLibrariesSbplSplineVisualization.hpp
    HEADERS
        MotionPlanningLibrariesStateVisualization.hpp
        MotionPlanningLibrariesSbplMprimsVisualization.hpp
        MotionPlanningLibrariesSbplSplineVisualization.hpp
    DEPS motion_planning_libraries
)

