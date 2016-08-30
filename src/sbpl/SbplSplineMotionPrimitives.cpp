#include "SbplSplineMotionPrimitives.hpp"
#include <iostream> //FIXME remove after debug

using namespace base::geometry;
namespace motion_planning_libraries 
{

SbplSplineMotionPrimitives::SbplSplineMotionPrimitives() {}
  
SbplSplineMotionPrimitives::SbplSplineMotionPrimitives(const SplinePrimitivesConfig& config) :
  config(config)
{
  radPerDiscreteAngle = (M_PI*2.0) / config.numAngles;
  primitivesByAngle.resize(config.numAngles);
  generatePrimitives(config);
}

std::vector<Eigen::Vector2i> SbplSplineMotionPrimitives::generateDestinationCells(const SplinePrimitivesConfig& config) const
{
  std::vector<Eigen::Vector2i> result;

  const double radiusSquared = std::pow(config.destinationCircleRadius, 2);
  //iterate over all cells in rectangle around circle.
  //add those cells inside the circle
  for(int x = -config.destinationCircleRadius; x < config.destinationCircleRadius; ++x)
  {
    for(int y = -config.destinationCircleRadius; y < config.destinationCircleRadius; ++y)
    {
      //0,0 is the start cell
      if(x == 0 && y == 0)
        continue;
      
      const double centerX = config.cellCenterOffset.x() + x;
      const double centerY = config.cellCenterOffset.y() + y;
      const double distanceSquared = centerX * centerX + centerY * centerY;

        if (distanceSquared <= radiusSquared) 
        {
          result.emplace_back(x, y);
        }
    }
  }
  return result;
}

void SbplSplineMotionPrimitives::generatePrimitives(const SplinePrimitivesConfig& config)
{
  std::vector<Eigen::Vector2i> destinaionCells = generateDestinationCells(config);
  
  for(int startAngle = 0; startAngle < config.numAngles; ++startAngle)
  {
    generatePrimitivesForAngle(startAngle, destinaionCells);
  }
}

void SbplSplineMotionPrimitives::generatePrimitivesForAngle(const int startAngle,
                                                            std::vector<Eigen::Vector2i> destinationCells)
{
  //NOTE since the destinationCells form a circle, we do not need to rotate them.
  int id = 0; //the "same" primitives should have the same id for each start angle
  const double radStartAngle = startAngle * radPerDiscreteAngle;
  
  for(const Eigen::Vector2i& dest : destinationCells)
  {
    const Eigen::Vector2d destRot = Eigen::Rotation2D<double>(-radStartAngle) * dest.cast<double>();
    //forward movement
    if(destRot.x() > config.cellCenterOffset.x())
    {
      SplinePrimitive prim = getPrimitive(startAngle, startAngle, dest, id);
      primitivesByAngle[startAngle].push_back(prim);
      ++id;
    }
    //lateral movement
    else if(destRot.x() == config.cellCenterOffset.x())
    {
      //TODO lateral movements
    }
    //backward movement
    else if(destRot.x() < config.cellCenterOffset.x())
    {
      //TODO backward movement
    }
  }  
  
  
}

SplinePrimitive SbplSplineMotionPrimitives::getPrimitive(const int startAngle,
                                                         const int endAngle,
                                                         const Eigen::Vector2i destination,
                                                         const int primId) const
{
  SplinePrimitive prim;
  
  
  const double radStartAngle = startAngle * radPerDiscreteAngle;
  const base::Vector2d start(config.cellCenterOffset * config.gridSize);
  const base::Vector2d startDirection = start + Eigen::Rotation2D<double>(radStartAngle) * Eigen::Vector2d::UnitX();
  

  
  const double radEndAngle = endAngle * radPerDiscreteAngle;
  const base::Vector2d end((destination.cast<double>() + config.cellCenterOffset) * config.gridSize);
  const base::Vector2d endDirection = end + Eigen::Rotation2D<double>(radEndAngle) * Eigen::Vector2d::UnitX();
  
  std::vector<base::Vector2d> points{start, startDirection, end, endDirection};
  std::vector<SplineBase::CoordinateType> types{SplineBase::ORDINARY_POINT,
                                                SplineBase::TANGENT_POINT_FOR_PRIOR,
                                                SplineBase::ORDINARY_POINT,
                                                SplineBase::TANGENT_POINT_FOR_PRIOR};
                                                
  prim.spline = Spline2(0.1, 4); //FIXME get from config?
  prim.spline.interpolate(points, std::vector<double>(), types);
  prim.startAngle = startAngle;
  prim.endAngle = endAngle;
  prim.endPosition = destination;
  prim.id = primId;
   

  
  return prim;
}

const std::vector<SplinePrimitive>& SbplSplineMotionPrimitives::getPrimitiveForAngle(const int angle) const
{
  assert(angle >= 0);
  assert(angle < config.numAngles);
  return primitivesByAngle[angle];
}

const SplinePrimitivesConfig& SbplSplineMotionPrimitives::getConfig() const
{
  return config;
}





}//end namespace motion_planning_libraries
