#ifndef _PLANNING_HELPERS_HPP_
#define _PLANNING_HELPERS_HPP_

#include <boost/shared_ptr.hpp>
//#include <CGAL/Point_2.h>

#include <envire/maps/TraversabilityGrid.hpp>

namespace motion_planning_libraries
{

typedef envire::TraversabilityGrid::ArrayType TravData;
    
class GridCalculations {
 
 private:
    envire::TraversabilityGrid* mpTravGrid;
    boost::shared_ptr<TravData> mpTravData;
    Eigen::Affine3d mFootprint2Grid;
    // Contains all coordinates within the local frame.
    std::vector< base::Vector3d > mFootprintLocal;
    
 public:    
 
    GridCalculations() : mpTravGrid(NULL),
            mpTravData(),
            mFootprint2Grid(),
            mFootprintLocal() {
    }
 
    void setTravGrid(envire::TraversabilityGrid* trav_grid, boost::shared_ptr<TravData> trav_data) {
        mpTravGrid = trav_grid; 
        mpTravData = trav_data;
    }
    
    void setFootprintRectangleInGrid(int rectangle_lenth_x, int rectangle_width_y) {
        if(mpTravGrid == NULL) {
            throw std::runtime_error("Trav Grid not set");
        }
        
        mFootprintLocal.clear();
        for(int y=-rectangle_width_y/2.0; y < std::ceil(rectangle_width_y/2.0); ++y) {
            for(int x=-rectangle_lenth_x/2.0; x < std::ceil(rectangle_lenth_x/2.0); ++x) {
                mFootprintLocal.push_back(base::Vector3d(x,y,0));
            }
        }
    }
    
    void setFootprintCircleInGrid(int radius_grid, bool  filled=true) {
        if(mpTravGrid == NULL) {
            throw std::runtime_error("Trav Grid not set");
        }
        
        mFootprintLocal.clear();
        
        if(filled) {
            base::Vector3d vec;
            int squared_radius = radius_grid * radius_grid;
            for(int y=-radius_grid; y < std::ceil(radius_grid) ; ++y) {
                for(int x=-radius_grid; x < std::ceil(radius_grid); ++x) {
                    vec[0] = x;
                    vec[1] = y;
                    vec[2] = 0.0;
                    if(vec.squaredNorm() <= squared_radius) {
                        mFootprintLocal.push_back(vec);
                    }
                }
            }
        } else {
            // Just draw the outline, should be used for footprint-planning.
            int num_vertices = 32;
            double rot = 2*M_PI/(double)num_vertices;
            base::Vector3d vec(radius_grid, 0.0, 0.0);
            // Add circle center
            mFootprintLocal.push_back(base::Vector3d(0.0, 0.0, 0.0));
            
            for(int i=0; i<num_vertices; ++i) {
                mFootprintLocal.push_back(vec);
                vec = Eigen::AngleAxisd(rot, Eigen::Vector3d::UnitZ()) * vec;
            }
        }
    }
    
    void setFootprintPoseInGrid(int footprint_x_grid, 
            int footprint_y_grid, 
            int footprint_theta_grid) {
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
        
        if(mFootprintLocal.empty()) {
            throw std::runtime_error("No footprint has been set.");
        }
        
        int fp_x = 0;
        int fp_y = 0;
        base::Vector3d result;
        uint8_t class_value = 0;
        double driveability = 0;
        
        std::vector<base::Vector3d>::iterator it = mFootprintLocal.begin(); 
        for(;it != mFootprintLocal.end(); ++it) {
            
            // Transform to grid frame.
            result = mFootprint2Grid * *it;
            fp_x = result[0];
            fp_y = result[1];
             
            // Check borders.
            if(     fp_x < 0 || fp_x >= (int)mpTravGrid->getCellSizeX() ||
                    fp_y < 0 || fp_y >= (int)mpTravGrid->getCellSizeY()) {
                LOG_DEBUG("State (%d,%d) is invalid (footprint (%4.2f,%4.2f) not within the grid)", 
                        mFootprint2Grid.translation()[0], mFootprint2Grid.translation()[1], fp_x, fp_y);
                return false;
            } 
            
            // Check obstacle.
            class_value = (*mpTravData)[fp_y][fp_x];
            driveability = (mpTravGrid->getTraversabilityClass(class_value)).getDrivability();
        
            if(driveability == 0.0) {
                LOG_DEBUG("State (%d,%d) is invalid (footprint (%4.2f,%4.2f) lies on an obstacle)", 
                        mFootprint2Grid.translation()[0], mFootprint2Grid.translation()[1], fp_x, fp_y);
                return false;
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
        
        if(mFootprintLocal.empty()) {
            throw std::runtime_error("No footprint has been set.");
        }
            
        int fp_x = 0;
        int fp_y = 0;
        base::Vector3d result;

        std::vector<base::Vector3d>::iterator it = mFootprintLocal.begin(); 
        for(;it != mFootprintLocal.end(); ++it) {
            
            // Transform to grid frame.
            result = mFootprint2Grid * *it;
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
};

} // end namespace motion_planning_libraries

#endif
