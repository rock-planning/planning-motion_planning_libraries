#pragma once
#include <base/Eigen.hpp>
#include <base/Spline.hpp>
#include <vector>

namespace motion_planning_libraries 
{

struct SplinePrimitivesConfig 
{
  double gridSize = 0.1; //width and height of a grid cell
  int numAngles = 16; //number of discrete start angles angles. A full set of primitives will be generated for each start angle (has to be even)
  int numEndAngles = 3; //number of possible end orientations per cell (numEndAngles <= numAngles/2) (has to be odd)

  /*Primitives will be generated leading to all cells withing this radius (in cells) */
  int destinationCircleRadius = 20;
  /**Offset of the center of a cell from the grid index. E.g. if index (0,0) denotes the
   * bottom left corner of the cell, the center offset should be (0.5, 0.5).
   * This value should always be given as if gridSize is 1. I.e. it is in percent of a grid cell*/
  base::Vector2d cellCenterOffset = base::Vector2d(0.5, 0.5);
};


/**Describes a single motion primitive. */
struct SplinePrimitive
{
  unsigned id; //FIXME why does a spline need an id?
  unsigned startAngle; //discretized start angle
  double startAngleRad; //start angle in rad
  unsigned endAngle; //discretized end angle
  double endAngleRad; //end angle in rad
  Eigen::Vector2i endPosition;//discret end position, i.e. the cell that this spline ends in
  base::geometry::Spline2 spline; //spline of the movement (not discretized), starts in the center of the first cell, ends in the center of the last cell
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
   *  at @p destination with discrete @p endAngle and id @p primId.*/
  SplinePrimitive getPrimitive(const int startAngle, const int endAngle,
                               const Eigen::Vector2i destination, const int primId) const;
                               
  /** Generates all end angles for a given start angle based on config.numEndAngles */                               
  std::vector<int> generateEndAngles(const int startAngle, const SplinePrimitivesConfig& config) const;          
  
  /** @throw std::runtime_error if config is not vaild */
  void validateConfig(const SplinePrimitivesConfig& config) const;
};


}