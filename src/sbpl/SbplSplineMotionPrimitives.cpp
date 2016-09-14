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
    //for cellSkipFactor < 1 the algorithm generates the same coordinates several times
    //therefore we put them in a set :-) This is simpler than writing a more complex algorithm
    std::set<std::pair<int, int>> coordinates; //use pair instead of Vector2i because operator< is implemented for pair
    for(int n = 0, r = 1; r < config.destinationCircleRadius; ++n, r = config.cellSkipFactor * n * n)
    {
        //see https://en.wikipedia.org/wiki/Midpoint_circle_algorithm
        if(r <= 0) //happens for cellSkipFactor < 1
            r = 1; 
        int x = r;
        int y = 0;
        int err = 0;

        while (x >= y)
        {
            coordinates.emplace(x,y);
            coordinates.emplace(y,x);
            coordinates.emplace(-y, x);
            coordinates.emplace(-x, y);
            coordinates.emplace(-x, -y);
            coordinates.emplace(-y, -x);
            coordinates.emplace(y, -x);
            coordinates.emplace(x,-y);

            y += 1;
            err += 1 + 2*y;
            if (2*(err-x) + 1 > 0)
            {
                x -= 1;
                err += 1 - 2*x;
            }
        }
    }
    
    std::vector<Eigen::Vector2i> result;
    for(const auto& point : coordinates)
    {
        result.emplace_back(point.first, point.second);
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
    
    //debug code below
    size_t count = 0;
    for(int angle = 0; angle < config.numAngles; ++angle)
    {
        count += getPrimitiveForAngle(angle).size();
    }
}

void SbplSplineMotionPrimitives::generatePrimitivesForAngle(const int startAngle,
                                                            std::vector<Eigen::Vector2i> destinationCells)
{
    /* Main idea:
     * For each destination cell: generate primitives from (0,0) to that cell.
     * The amount of primitives is defined by the config.
     * Additionally generate point turn primitives from (0,0) to (0,0)  
     */
    
    //NOTE since the destinationCells form a circle, we do not need to rotate them.
    int id = 0; //the "same" primitives should have the same id for each start angle according to sbpl
    const double radStartAngle = startAngle * radPerDiscreteAngle;
    const double epsilon = 0.1; //makes the distinction between forward/backward/lateral easier
        
    for(const Eigen::Vector2i& dest : destinationCells)
    { 
        //rotate destination for easier checks. If it is rotated we just need to check
        //whether destRot.x is above 0 (cellCenterOffset) to know if this is a forward or backward motion
        const Eigen::Vector2d destRot = Eigen::Rotation2D<double>(-radStartAngle) * (dest.cast<double>() + config.cellCenterOffset);

        //forward and backward movements
        const std::vector<int> endAngles = generateEndAngles(startAngle, config);
        for(int endAngle : endAngles)
        {
            //forward movement
            if(config.generateForwardMotions &&
               destRot.x() > config.cellCenterOffset.x() + epsilon)
            {
                SplinePrimitive prim = getPrimitive(startAngle, endAngle, dest,
                                                    id, SplinePrimitive::SPLINE_MOVE_FORWARD);
                primitivesByAngle[startAngle].push_back(prim);
                ++id;
            }
            //backward movement
            else if(config.generateBackwardMotions &&
                    destRot.x() < config.cellCenterOffset.x() - epsilon)
            {
                //the robot is driving backwards. we calculate the spline as if the robot is
                //rotated by 180° and change the start and end rotation afterwards.
                const int oppositStartAngle = (startAngle + config.numAngles / 2) % config.numAngles;
                const int oppositEndAngle = (endAngle + config.numAngles / 2) % config.numAngles;
                SplinePrimitive prim = getPrimitive(oppositStartAngle, oppositEndAngle,
                                                    dest, id, SplinePrimitive::SPLINE_MOVE_BACKWARD);
                prim.startAngle = startAngle;
                prim.endAngle = endAngle; //FIXME use endangle
                prim.startAngleRad = startAngle * radPerDiscreteAngle;
                prim.endAngleRad = endAngle * radPerDiscreteAngle;
                primitivesByAngle[startAngle].push_back(prim);
                ++id;
            }
        }
        //lateral movement
        if(config.generateLateralMotions &&
           destRot.x() <= config.cellCenterOffset.x() + epsilon &&
           destRot.x() >= config.cellCenterOffset.x() - epsilon)
        {
            //the robot is driving sidewards. spline is calculated as if the robot is rotated 90° 
            // and moving forward. angles are fixed afterwards
            const int leftStartAngle = (startAngle + config.numAngles / 4) % config.numAngles;
            const int rightStartAngle = (startAngle - config.numAngles / 4) % config.numAngles;
            
            SplinePrimitive leftPrim = getPrimitive(leftStartAngle, leftStartAngle,
                                                    dest, id, SplinePrimitive::SPLINE_MOVE_LATERAL);
            leftPrim.startAngle = startAngle;
            leftPrim.endAngle = startAngle;
            leftPrim.endAngleRad = radStartAngle;
            leftPrim.startAngleRad = radStartAngle;
            primitivesByAngle[startAngle].push_back(leftPrim);
            ++id;
            
            SplinePrimitive rightPrim = getPrimitive(rightStartAngle, rightStartAngle,
                                                    dest, id, SplinePrimitive::SPLINE_MOVE_LATERAL);
            
            
            rightPrim.startAngle = startAngle;
            rightPrim.endAngle = startAngle;
            rightPrim.endAngleRad = radStartAngle;
            rightPrim.startAngleRad = radStartAngle;
            primitivesByAngle[startAngle].push_back(rightPrim);
            ++id;
        }
    }
    
    if(config.generatePointTurnMotions)
    {
        //point turns are a special case
        for(int angle = 0; angle < config.numAngles; ++ angle)
        {
            if(angle == startAngle)
                continue;
            
            SplinePrimitive prim;
            prim.startAngle = startAngle;
            prim.endAngle = angle;
            prim.startAngleRad = radStartAngle;
            prim.endAngleRad = angle * radPerDiscreteAngle;
            prim.id = id;
            prim.motionType = SplinePrimitive::SPLINE_POINT_TURN;
            prim.endPosition << 0, 0;
            primitivesByAngle[startAngle].push_back(prim);
            ++id;
        }
    }
    
}

SplinePrimitive SbplSplineMotionPrimitives::getPrimitive(const int startAngle,
                                                         const int endAngle,
                                                         const Eigen::Vector2i destination,
                                                         const int primId,
                                                         const SplinePrimitive::Type& type) const
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
                                                    
    prim.spline = Spline2(config.splineGeometricResolution, config.splineOrder);
    prim.spline.interpolate(points, std::vector<double>(), types);
    prim.startAngle = startAngle;
    prim.endAngleRad = radEndAngle;
    prim.startAngleRad = radStartAngle;
    prim.endAngle = endAngle;
    prim.endPosition = destination;
    prim.id = primId;
    prim.motionType = type;

    return prim;
}

std::vector<int> SbplSplineMotionPrimitives::generateEndAngles(const int startAngle, const SplinePrimitivesConfig& config) const
{
    //otherwise the calculation below gets more complicated
    assert(config.numAngles % 2 == 0);
    
    std::vector<int> result;
    result.push_back(startAngle);
    //the left/right distinction is necessary in case of an even number of end angles
    int numAnglesLeftSide = (config.numEndAngles - 1) / 2;
    int numAnglesRightSide = (config.numEndAngles - 1) / 2;
    
    if(config.numEndAngles % 2 == 0)
    {
        ++numAnglesRightSide;
    }
    if(numAnglesLeftSide > 0)
    {
        const double leftStep = config.numAngles / 4.0 / numAnglesLeftSide;
        const double rightStep = config.numAngles / 4.0 / numAnglesRightSide;
        
        int count = 0;
        for(double angle = startAngle - leftStep; count < numAnglesLeftSide; ++count, angle -= leftStep)
        {
            //casting angle to int before adding config-numAngles is important to get symmetric results
            //+ config.numAngles is done to avoid negative number modulo (which is implementation defined in c++03)
            const int realAngle = (((int)angle) + config.numAngles) % config.numAngles;
            result.push_back(realAngle);
        }
        count = 0;
        for(double angle = startAngle + rightStep; count < numAnglesRightSide; ++count, angle += rightStep)
        {
            const int realAngle = (((int)angle) + config.numAngles) % config.numAngles;
            result.push_back(realAngle);
        }
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
        
    if(config.numEndAngles > config.numAngles / 2)
        throw std::runtime_error("numEndAngles has to be <= numAngles / 2");
    
    if(config.splineGeometricResolution <= 0)
        throw std::runtime_error("splineGeometricResolution has to be > 0");
    
    if(config.splineOrder < 3)
        throw std::runtime_error("splineOrder has to be >= 3");
}



}//end namespace motion_planning_libraries
