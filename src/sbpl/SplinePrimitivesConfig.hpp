namespace motion_planning_libraries 
{
struct SplinePrimitivesConfig 
{
    double gridSize; //width and height of a grid cell
    int numAngles; //number of discrete start angles angles. A full set of primitives will be generated for each start angle (has to be even)
    
    /** maximum number of possible end orientations per cell.
     * Has to be <= numAngles/2.
     * Has to be odd.
     * @note This is an upper boundary. Depending on the value of numAngles, it might not be reached*/
    int numEndAngles; 

    /*Maximum radius in which primitives will be generated (in cells) */
    int destinationCircleRadius;
    
    /**Influences how sparse the cells are distributed inside the destination circle. bigger number means more sparse*/
    double cellSkipFactor;
    
    /**Offset of the center of a cell from the grid index. E.g. if index (0,0) denotes the
    * bottom left corner of the cell, the center offset should be (0.5, 0.5).
    * This value should always be given as if gridSize is 1. I.e. it is in percent of a grid cell*/
    base::Vector2d cellCenterOffset;
    
    //resolution of the internal splines
    double splineGeometricResolution;
    //order if the internal splines
    double splineOrder;
    
    bool generateForwardMotions;
    bool generateBackwardMotions;
    bool generateLateralMotions;
    bool generatePointTurnMotions;
    
    SplinePrimitivesConfig() : gridSize(0.1), numAngles(16), numEndAngles(7),
                               destinationCircleRadius(20), cellSkipFactor(0.3),
                               cellCenterOffset(0.5, 0.5), splineGeometricResolution(0.1),
                               splineOrder(4), generateForwardMotions(true),
                               generateBackwardMotions(true), generateLateralMotions(true),
                               generatePointTurnMotions(true){}
};
}