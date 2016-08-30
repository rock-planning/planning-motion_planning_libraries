#pragma once
#include <base/Eigen.hpp>
#include <motion_planning_libraries/Config.hpp>
#include <base/Spline.hpp>
#include <vector>

namespace motion_planning_libraries 
{

struct SplinePrimitivesConfig 
{
  double gridSize = 0.1; //width and height of a grid cell
  int numAngles = 16; //number of discrete start angles angles. A full set of primitives will be generated for each start angle
  int numEndAngles = 3; //number of possible end orientations per cell (has to be odd)

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
  //TODO wie macht man das mit der startposition der spline?
  base::geometry::Spline2 spline; //spline of the movement (not discretized), starts in the center of the first cell, ends in the center of the last cell
  MovementType movType; //FIXME not sure if I need this,
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
  SbplSplineMotionPrimitives(const SplinePrimitivesConfig& config);
  
  const std::vector<SplinePrimitive>& getPrimitiveForAngle(const int angle) const;
  const SplinePrimitivesConfig& getConfig() const;
private:
  void generatePrimitives(const SplinePrimitivesConfig& config);
  
  /** @p startAngle defines the starting orientation of the robot. Is discrete */
  void generatePrimitivesForAngle(const int startAngle, std::vector<Eigen::Vector2i> destinationCells);
  std::vector<Eigen::Vector2i> generateDestinationCells(const SplinePrimitivesConfig& config) const;  
  SplinePrimitive getPrimitive(const int startAngle, const int endAngle,
                               const Eigen::Vector2i destination, const int primId) const;
};


}