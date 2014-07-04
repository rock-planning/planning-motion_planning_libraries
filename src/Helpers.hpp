#ifndef _PLANNING_HELPERS_HPP_
#define _PLANNING_HELPERS_HPP_

#include <boost/shared_ptr.hpp>

#include <envire/maps/TraversabilityGrid.hpp>

namespace motion_planning_libraries
{

typedef envire::TraversabilityGrid::ArrayType TravData;
    
class GridCalculations {
 
 private:
    Eigen::Affine3d mFootprint2Grid;
    double mFootprintLengthGrid;
    double mFootprintWidthGrid;
    double mFootprintLengthGrid2;
    double mFootprintWidthGrid2;
    envire::TraversabilityGrid* mpTravGrid;
    boost::shared_ptr<TravData> mpTravData;
    
 public:    
 
    GridCalculations() : mFootprint2Grid(),
            mFootprintLengthGrid(0),
            mFootprintWidthGrid(0),
            mFootprintLengthGrid2(0),
            mFootprintWidthGrid2(0),
            mpTravGrid(NULL),
            mpTravData() {
    }
 
    void setTravGrid(envire::TraversabilityGrid* trav_grid, boost::shared_ptr<TravData> trav_data) {
        mpTravGrid = trav_grid; 
        mpTravData = trav_data;
        // Creates a local copy of the grid data.
        // fails:
        //mpTravData = trav_grid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY);
        //mpTravData = boost::shared_ptr<TravData>(new TravData(trav_grid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY)));   
    }
    
    void setFootprint(double footprint_x_grid, 
            double footprint_y_grid, 
            double footprint_theta_grid, 
            double footprint_length_grid, 
            double footprint_width_grid) {
        mFootprintLengthGrid = footprint_length_grid;
        mFootprintWidthGrid = footprint_width_grid;
        mFootprintLengthGrid2 = footprint_length_grid / 2.0;
        mFootprintWidthGrid2 = footprint_width_grid / 2.0;
        mFootprint2Grid.setIdentity();
        mFootprint2Grid.rotate(Eigen::AngleAxis<double>(footprint_theta_grid, base::Vector3d(0.0, 0.0, 1.0)));
        mFootprint2Grid.translation() = base::Vector3d(footprint_x_grid, footprint_y_grid, 0.0);    
    }
    
    /**
     * Checks if the current footprint is valid. Some pixels within the rectangle
     * may not be checked. Problem?
     */
    bool isValid() {
    
        if(mpTravGrid == NULL) {
            throw std::runtime_error("Trav Grid not set");
        }
    
        int fp_x = 0;
        int fp_y = 0;
        base::Vector3d result;
        int class_value = 0.0;
        double driveability = 0.0;
        
        for(double x = 0; x < mFootprintLengthGrid; ++x) {
            for(double y = 0; y < mFootprintWidthGrid; ++y) {
                result = mFootprint2Grid * base::Vector3d(x-mFootprintLengthGrid2, y-mFootprintWidthGrid2, 0.0);
                fp_x = result[0];
                fp_y = result[1];
                
                // Check borders.
                if(     fp_x < 0 || fp_x >= (int)mpTravGrid->getCellSizeX() ||
                        fp_y < 0 || fp_y >= (int)mpTravGrid->getCellSizeY()) {
                    base::Vector3d vec = mFootprint2Grid.translation();
                    LOG_DEBUG("State (%d,%d) is invalid (footprint (%4.2f,%4.2f) not within the grid)", 
                            vec[0], vec[1], fp_x, fp_y);
                    return false;
                } 
                
                // Check obstacle.
                class_value = (*mpTravData)[fp_y][fp_x];
                driveability = (mpTravGrid->getTraversabilityClass(class_value)).getDrivability();
            
                if(driveability == 0.0) {
                    base::Vector3d vec = mFootprint2Grid.translation();
                    LOG_DEBUG("State (%d,%d) is invalid (footprint (%4.2f,%4.2f) lies on an obstacle)", 
                            vec[0], vec[1], fp_x, fp_y);
                    return false;
                }  
            }
        }
        return true;
    }
    
    /**
     * Sets the current footprint to the passed trav-class. Uses x,y+=0.5
     * to be sure to set every pixel.
     */ 
    void setValue(int cost_class) {
    
        if(mpTravGrid == NULL) {
            throw std::runtime_error("Trav Grid not set");
        }
            
        int fp_x = 0;
        int fp_y = 0;
        base::Vector3d result;
    
        for(double x = 0; x < mFootprintLengthGrid; x+=0.5) {
            for(double y = 0; y < mFootprintWidthGrid; y+=0.5) {
                result = mFootprint2Grid * base::Vector3d(x-mFootprintLengthGrid2, y-mFootprintWidthGrid2, 0.0);
                fp_x = result[0];
                fp_y = result[1];
                
                // Check borders.
                if(     fp_x < 0 || fp_x >= (int)mpTravGrid->getCellSizeX() ||
                        fp_y < 0 || fp_y >= (int)mpTravGrid->getCellSizeY()) {
                    continue;
                } 
                
                (*mpTravData)[fp_y][fp_x] = cost_class;
                mpTravGrid->setProbability(1.0, fp_x, fp_y);
            }
        }
    }
};

} // end namespace motion_planning_libraries

#endif
