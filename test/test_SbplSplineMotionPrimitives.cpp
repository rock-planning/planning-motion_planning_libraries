#include <boost/test/unit_test.hpp>
#define private public //need to be able to test private members 
#define protected public
#include <motion_planning_libraries/sbpl/SbplSplineMotionPrimitives.hpp>
#include <iostream>
#include <fstream>
#include <map>
#include <set>


using namespace motion_planning_libraries;
using namespace std;

#define VECTOR_CONTAINS(vec, item) \
  BOOST_CHECK(find(vec.begin(), vec.end(), item) != vec.end())
  

BOOST_AUTO_TEST_CASE(numEndAngles_1)
{
    SplinePrimitivesConfig config;
    config.generateBackwardMotions = false;
    config.generateLateralMotions = false;
    config.generatePointTurnMotions = false;
    config.numAngles = 10;    
    config.numEndAngles = 1;
    
    SbplSplineMotionPrimitives prims(config);
    
    for(int i = 0; i < config.numAngles; ++i)
    {
        BOOST_CHECK(prims.getPrimitiveForAngle(i).size() > 0);
    }
}


BOOST_AUTO_TEST_CASE(generate_end_angles)
{
    SplinePrimitivesConfig config;
    config.generateBackwardMotions = false;
    config.generateLateralMotions = false;
    config.generatePointTurnMotions = false;
    config.numAngles = 32;    
    config.numEndAngles = 15;
    
    SbplSplineMotionPrimitives prims(config);    
    for(int i = 0; i < config.numAngles; ++i)
    {
        vector<int> endAngles = prims.generateEndAngles(i, config);
        BOOST_CHECK(endAngles.size() <= config.numEndAngles);
        
        set<int> angleSet;
        for(int angle : endAngles)
        {
            angleSet.insert(angle);
        }
        BOOST_CHECK(angleSet.size() == endAngles.size());
    }
}

BOOST_AUTO_TEST_CASE(no_cells_outside_radius)
{
    SplinePrimitivesConfig config;
    SbplSplineMotionPrimitives primGen(config);
    for(int i = 0; i < config.numAngles; ++i)
    {
        const vector<SplinePrimitive>& prims = primGen.getPrimitiveForAngle(i);
        for(const SplinePrimitive& prim : prims)
        {
            const double dist = prim.endPosition.norm();
            BOOST_CHECK(dist <= config.destinationCircleRadius);
        }
    }
}







//TODO test that end angles are unique
