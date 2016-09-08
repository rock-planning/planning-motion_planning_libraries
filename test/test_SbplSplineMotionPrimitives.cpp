#include <boost/test/unit_test.hpp>
#define private public //need to be able to test private members 
#define protected public
#include <motion_planning_libraries/sbpl/SbplSplineMotionPrimitives.hpp>
#include <iostream>
#include <fstream>
#include <map>
#include <set>


using namespace motion_planning_libraries;

#define VECTOR_CONTAINS(vec, item) \
  BOOST_CHECK(std::find(vec.begin(), vec.end(), item) != vec.end())
  

BOOST_AUTO_TEST_CASE(test_generateDestinationCells)
{
           
//   SplinePrimitivesConfig config;
//   config.destinationCircleRadius = 1;
//   config.cellSkipFactor = 0.1;
//   
//   
//   SbplSplineMotionPrimitives prims(config);
//   
//   std::vector<Eigen::Vector2i> destinations = prims.generateDestinationCells(config);
//   std::ofstream myfile;
//   myfile.open ("/home/arne/spline2.dat");
//   for(Eigen::Vector2i& i : destinations)
//     myfile << i.transpose() << std::endl;
//   myfile.close();
// //   VECTOR_CONTAINS(destinations, Eigen::Vector2i(0, 0));
  
  
}

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
        std::vector<int> endAngles = prims.generateEndAngles(i, config);
        BOOST_CHECK(endAngles.size() <= config.numEndAngles);
        
        std::set<int> angleSet;
        for(int angle : endAngles)
        {
            angleSet.insert(angle);
        }
        BOOST_CHECK(angleSet.size() == endAngles.size());
    }
}




//TODO test that end angles are unique
