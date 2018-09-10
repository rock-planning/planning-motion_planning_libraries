#pragma once
#include <base/Eigen.hpp>
#include <base/Spline.hpp>
#include <vector>
#include <set>
#include "SplinePrimitivesConfig.hpp"

namespace motion_planning_libraries 
{



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
    Eigen::Vector2i endPosition;//discrete end position, i.e. the cell that this spline ends in
    //spline of the movement (not discretized), starts in the center of the first cell, ends in the center of the last cell
    //NOTE for point turns the spline is invalid 
    base::geometry::Spline2 spline; 
};


/**Generates sbpl motion primitves using b-splines.
 * Creates motion primitves for 2d planning (i.e. x,y,orientation) */
class SbplSplineMotionPrimitives
{
private:
    std::vector< std::vector<SplinePrimitive> > primitivesByAngle; //primitives by startAngle by id
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
