#include <boost/test/unit_test.hpp>
#define private public //need to be able to test private members 
#define protected public
#include <motion_planning_libraries/sbpl/SbplSplineMotionPrimitives.hpp>
#include <iostream>
#include <fstream>


using namespace motion_planning_libraries;

#define VECTOR_CONTAINS(vec, item) \
  BOOST_CHECK(std::find(vec.begin(), vec.end(), item) != vec.end())
  

BOOST_AUTO_TEST_CASE(test_generateDestinationCells)
{
           
  SplinePrimitivesConfig config;
  config.destinationCircleRadius = 10;
  config.cellSkipFactor = 0.1;
  
  
  SbplSplineMotionPrimitives prims(config);
  
  std::vector<Eigen::Vector2i> destinations = prims.generateDestinationCells(config);
  std::ofstream myfile;
  myfile.open ("/home/arne/spline2.dat");
  for(Eigen::Vector2i& i : destinations)
    myfile << i.transpose() << std::endl;
  myfile.close();
//   VECTOR_CONTAINS(destinations, Eigen::Vector2i(0, 0));
  
  
}