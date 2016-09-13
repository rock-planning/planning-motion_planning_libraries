#pragma once
#include <base/Eigen.hpp>
#include <base/Spline.hpp>
#include <vector>
#include <set>

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


/**Describes a single motion primitive. */
struct SplinePrimitive
{
    enum Type 
    {
        SPLINE_MOVE_FORWARD,
        SPLINE_MOVE_BACKWARD,
        SPLINE_MOVE_LATERAL,
        SPLINE_POINT_TURN
    }motionType;
    
    unsigned id; //FIXME why does a spline need an id?
    unsigned startAngle; //discretized start angle in [0 .. numAngles[
    double startAngleRad; //start angle in rad
    unsigned endAngle; //discretized end angle [0 .. numAngles[
    double endAngleRad; //end angle in rad
    Eigen::Vector2i endPosition;//discret end position, i.e. the cell that this spline ends in
    //spline of the movement (not discretized), starts in the center of the first cell, ends in the center of the last cell
    //NOTE for point turns the spline is invalid 
    base::geometry::Spline2 spline; 
};


/**Generates sbpl motion primitves using b-splines.
 * Creates motion primitves for 2d planning (i.e. x,y,orientation) */
class SbplSplineMotionPrimitives
{
private:
    std::vector<std::vector<SplinePrimitive>> primitivesByAngle; //primitives by startAngle by id
    SplinePrimitivesConfig config;
    double radPerDiscreteAngle;
  
public:
    
    SbplSplineMotionPrimitives();
    /** @throw std::runtime_error if config is not vaild */
    SbplSplineMotionPrimitives(const SplinePrimitivesConfig& config);
    
    /** @param angle has to be < config.numAngles and > 0 */
    const std::vector<SplinePrimitive>& getPrimitiveForAngle(const int angle) const;
    const SplinePrimitivesConfig& getConfig() const;
private:
    void generatePrimitives(const SplinePrimitivesConfig& config);
    
    /** @param startAngle defines the starting orientation of the robot. Is discrete.
    *  @param destinationCells indices of cells that the primitives should go to */
    void generatePrimitivesForAngle(const int startAngle, std::vector<Eigen::Vector2i> destinationCells);
    
    /**Generates a circular field of destination cells based on config.destinationCircleRadius */ 
    std::vector<Eigen::Vector2i> generateDestinationCells(const SplinePrimitivesConfig& config) const;
    
    /** Creates a primitiv starting at (0,0) with a discrete @p startAngle, ending 
    *  at @p destination with discrete @p endAngle and id @p primId of @p type.*/
    SplinePrimitive getPrimitive(const int startAngle, const int endAngle,
                                const Eigen::Vector2i destination, const int primId,
                                const SplinePrimitive::Type& type) const;
                                
    /** Generates all end angles for a given start angle based on config.numEndAngles */                               
    std::vector<int> generateEndAngles(const int startAngle, const SplinePrimitivesConfig& config) const;          
    
    /** @throw std::runtime_error if config is not vaild */
    void validateConfig(const SplinePrimitivesConfig& config) const;
};


}