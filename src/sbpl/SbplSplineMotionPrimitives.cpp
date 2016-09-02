#include "SbplSplineMotionPrimitives.hpp"
#include <iostream> //FIXME remove after debug
#include <set>

using namespace base::geometry;
namespace motion_planning_libraries 
{

SbplSplineMotionPrimitives::SbplSplineMotionPrimitives() {}
  
SbplSplineMotionPrimitives::SbplSplineMotionPrimitives(const SplinePrimitivesConfig& config) :
  config(config)
{
  validateConfig(config);
  
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
  const double epsilon = 0.1;
  const int numForwardAngles = config.numAngles / 2;
  assert(config.numEndAngles <= numForwardAngles);
  
    
  for(const Eigen::Vector2i& dest : destinationCells)
  {
    //rotate destination for easier checks
    const Eigen::Vector2d destRot = Eigen::Rotation2D<double>(-radStartAngle) * (dest.cast<double>() + config.cellCenterOffset);

    //forward and backward movements
    const std::vector<int> endAngles = generateEndAngles(startAngle, config);
    for(int endAngle : endAngles)
    {
      //forward movement
      if(destRot.x() > config.cellCenterOffset.x() + epsilon)
      {
        SplinePrimitive prim = getPrimitive(startAngle, endAngle, dest, id);
        primitivesByAngle[startAngle].push_back(prim);
        ++id;
      }
      //backward movement
      else if(destRot.x() < config.cellCenterOffset.x() - epsilon)
      {
        //the robot is driving backwards. we calculate the spline as if the robot is
        //rotated by 180° and change the start and end rotation afterwards.
        const int oppositStartAngle = (startAngle + config.numAngles / 2) % config.numAngles;
        const int oppositEndAngle = (endAngle + config.numAngles / 2) % config.numAngles;
        SplinePrimitive prim = getPrimitive(oppositStartAngle, oppositEndAngle, dest, id);
        prim.startAngle = startAngle;
        prim.endAngle = endAngle; //FIXME use endangle
        prim.startAngleRad = startAngle * radPerDiscreteAngle;
        prim.endAngleRad = endAngle * radPerDiscreteAngle;
        primitivesByAngle[startAngle].push_back(prim);
        ++id;
      }
    }
    //lateral movement
    if(destRot.x() <= config.cellCenterOffset.x() + epsilon &&
       destRot.x() >= config.cellCenterOffset.x() - epsilon)
      {
        //the robot is driving sidewards. spline is calculated as if the robot is rotated 90° 
        // and moving forward. angles are fixed afterwards
        const int leftStartAngle = (startAngle + config.numAngles / 4) % config.numAngles;
        const int rightStartAngle = (startAngle - config.numAngles / 4) % config.numAngles;
        
        SplinePrimitive leftPrim = getPrimitive(leftStartAngle, leftStartAngle, dest, id);
        leftPrim.startAngle = startAngle;
        leftPrim.endAngle = startAngle;
        leftPrim.endAngleRad = radStartAngle;
        leftPrim.startAngleRad = radStartAngle;
        primitivesByAngle[startAngle].push_back(leftPrim);
        ++id;
        
        SplinePrimitive rightPrim = getPrimitive(rightStartAngle, rightStartAngle, dest, id);
        rightPrim.startAngle = startAngle;
        rightPrim.endAngle = startAngle;
        rightPrim.endAngleRad = radStartAngle;
        rightPrim.startAngleRad = radStartAngle;
        primitivesByAngle[startAngle].push_back(rightPrim);
        ++id;
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
  prim.endAngleRad = radEndAngle;
  prim.startAngleRad = radStartAngle;
  prim.endAngle = endAngle;
  prim.endPosition = destination;
  prim.id = primId;

  return prim;
}

std::vector<int> SbplSplineMotionPrimitives::generateEndAngles(const int startAngle, const SplinePrimitivesConfig& config) const
{
  //+ config.numAngles is done to avoid negative number modulo (which is implementation defined in c++03)
  std::vector<int> result;
  result.push_back(startAngle);
  const int numAnglesPerSide = (config.numEndAngles - 1) / 2;
  const int step = config.numAngles / 4 / numAnglesPerSide;
  const int firstEndAngle = startAngle - config.numAngles / 4;
  const int lastEndAngle = startAngle + config.numAngles / 4;
  
  for(int angle = firstEndAngle; angle < startAngle; angle += step)
  {
    const int realAngle = (angle + config.numAngles) % config.numAngles;
    result.push_back(realAngle);
  }
  for(int angle = lastEndAngle; angle > startAngle; angle -= step)
  {
    const int realAngle = (angle + config.numAngles) % config.numAngles;
    result.push_back(realAngle);
  }
  return result;
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

void SbplSplineMotionPrimitives::validateConfig(const SplinePrimitivesConfig& config) const
{
  if(config.gridSize <= 0)
    throw std::runtime_error("gridSize has to be > 0");
  
  if(config.numAngles <= 0)
    throw std::runtime_error("numAngles has to be > 0");
  
  if(config.numEndAngles <= 0)
    throw std::runtime_error("numEndAngles has to be > 0");
  
  if(config.destinationCircleRadius <= 0)
    throw std::runtime_error("destinationCircleRadius has to be > 0");
  
  if(config.numAngles % 2 != 0)
    throw std::runtime_error("numAngles has to be even");
  
  if(config.numEndAngles % 2 == 0)
    throw std::runtime_error("numAngles has to be odd");
  
  if(config.numEndAngles > config.numAngles / 2)
    throw std::runtime_error("numEndAngles has to be <= numAngles / 2");
}



}//end namespace motion_planning_libraries
