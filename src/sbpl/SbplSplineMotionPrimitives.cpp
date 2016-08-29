#include "SbplSplineMotionPrimitives.hpp"
#include <iostream> //FIXME remove after debug

namespace motion_planning_libraries 
{

SbplSplineMotionPrimitives::SbplSplineMotionPrimitives(const SplinePrimitivesConfig& config) :
  config(config)
{
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

std::vector<Eigen::Vector2i> SbplSplineMotionPrimitives::getPixelsOnLine(const int x,
                                                                         const int yStart,
                                                                         const int yEnd) const
{
  std::vector<Eigen::Vector2i> result;
  for(int y = yStart; y <= yEnd; ++y)
  {
    result.emplace_back(x, y);
  }
  return result;
}


void SbplSplineMotionPrimitives::generatePrimitives(const SplinePrimitivesConfig& config)
{
  std::vector<Eigen::Vector2i> destinaionCells = generateDestinationCells(config);
  
//   for(int startAngle = 0; startAngle < config.numAngles; ++startAngle)
//   {
//     generatePrimitivesForAngle(startAngle);
//   }
}

void SbplSplineMotionPrimitives::generatePrimitivesForAngle(const unsigned startAngle)
{
//   const Vector2d start(0, 0);
//   const double startSlope = 0; //TODO calulate
//   const std::vector<base::Vector2d> cells = calculateDestinationsForAngle(startAngle);
  
}

// std::vector<base::Vector2d> SbplSplineMotionPrimitives::calculateDestinationsForAngle0(const unsigned int startAngle) const
// {
// 
// }


}//end namespace motion_planning_libraries
