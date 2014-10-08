#ifndef _MOTION_PLANNING_LIBRARIES_SBPL_MOTION_PRIMITIVES_HPP_
#define _MOTION_PLANNING_LIBRARIES_SBPL_MOTION_PRIMITIVES_HPP_

#include <vector>

#include <base/Eigen.hpp>

namespace motion_planning_libraries {
    
typedef Eigen::Matrix<int, 3, 1, Eigen::DontAlign> Vector3i;

/**
 * Speed in m/sec or rad/sec.
 */
struct MotionPrimitivesConfig {
    MotionPrimitivesConfig() :
            mSpeedForward(0.0),
            mSpeedBackward(0.0),
            mSpeedLateral(0.0),
            mSpeedTurn(0.0),
            mSpeedPointTurn(0.0),
            mMultiplierForward(1),
            mMultiplierBackward(1),
            mMultiplierLateral(1),
            mMultiplierTurn(1),
            mMultiplierPointTurn(1),
            mNumTurnPrimitives(0),
            mNumIntermediatePoses(0),
            mNumAngles(0),
            mMapWidth(0),
            mMapHeight(0),
            mGridSize(0.0) {   
    }
    
  public:
    double mSpeedForward;
    double mSpeedBackward;
    double mSpeedLateral;
    double mSpeedTurn;
    double mSpeedPointTurn;
    unsigned int mMultiplierForward;
    unsigned int mMultiplierBackward;
    unsigned int mMultiplierLateral;
    unsigned int mMultiplierTurn;
    unsigned int mMultiplierPointTurn;
    unsigned int mNumTurnPrimitives;
    
    unsigned int mNumIntermediatePoses; // Number of points a primitive consists of.
    unsigned int mNumAngles; // Number of discrete angles (in general 2*M_PI / 16)
    
    unsigned int mMapWidth;
    unsigned int mMapHeight;
    double mGridSize; // Width/length of a grid cell in meter.
};

/**
 * Describes a single motion primitive.
 */
struct Primitive {
 public:
    Primitive() : mId(0), mStartAngle(0), mEndPose(), 
            mCostMultiplier(0), mIntermediatePoses() 
    {
    }
    
    Primitive(int id, int start_angle, Vector3i end_pose, unsigned int cost_multiplier) : 
            mId(id), mStartAngle(start_angle), mEndPose(end_pose), 
            mCostMultiplier(cost_multiplier), mIntermediatePoses() 
    {
    }
     
    int mId;
    int mStartAngle;
    Vector3i mEndPose;
    unsigned int mCostMultiplier;
    std::vector<base::Vector3d> mIntermediatePoses;
};

/**
 * Allows to create and store the motion primitives file which is required
 * by SBPL for x,y,theta planning. x = forward, y = left, z = up
 * Each motion primitive consists of the following components:
 *  - ID of the motion primitive from 0 to mNumTurnPrimitives - 1
 *  - Discrete start angle
 *  - Discrete end pose (x_grid, y_grid, theta) theta has to be +- 0 to mNumAngles
 *  - Cost multiplier
 *  - mNumIntermediatePoses intermediate poses including start and end with (x_m, y_m, theta_rad)
 */
class SbplMotionPrimitives {
 public:
    
    SbplMotionPrimitives(struct MotionPrimitivesConfig config) {
    }
    
    ~SbplMotionPrimitives() {
    }
    
    /**
     * First uses fillListEndposes() to calculate the end poses in the world for 
     * all angles. These poses are discretized by rounding down (this should 
     * take care that we do not exceed the max turning radius.
     */
    void createPrimitives() {
        fillListEndposes();
        createIntermediatePoses();
    }
    
    void storeToFile(std::string path) {
    }
    
 private:
     struct MotionPrimitivesConfig mConfig;
     std::vector<struct Primitive> mListPrimitives;
     std::vector<base::Vector3d> mListEndposes; // x_m, y_m, theta_rad
     std::vector<Vector3i> mListDiscreteEndposes; // x_grid, y_grid, theta_step
     double mScaleFactor;
     
     /**
      * Calculates the end poses for 0 radians in x_m, y_m, theta_rad which can be 
      * reached within 'mScaleFactor' seconds. After that these poses are rotated
      * mNumAngles-1 times to cover the complete 2*M_PI.
      */
     void fillListEndposes() {
         // end pose and multiplier
         typedef std::pair<base::Vector3d, int> endpose;
         std::vector< endpose > poses_zero_rad; // x_m, y_m, theta_rad
         
         // Scaling: Each movement has to reach a new discrete state, so we have
         // to find the min scale factor. This represents the required movement time
         // to create these primitives.
         mScaleFactor = mConfig.mGridSize / mConfig.mSpeedForward;
         mScaleFactor = std::max(mScaleFactor, mConfig.mGridSize / mConfig.mSpeedBackward);
         mScaleFactor = std::max(mScaleFactor, mConfig.mGridSize / mConfig.mSpeedLateral);
         mScaleFactor = std::max(mScaleFactor, (2*M_PI/mConfig.mNumAngles) / mConfig.mSpeedPointTurn);
         mScaleFactor = std::max(mScaleFactor, (2*M_PI/mConfig.mNumAngles) / mConfig.mSpeedTurn);
         
         // Forward
         poses_zero_rad.push_back(endpose(base::Vector3d(mConfig.mSpeedForward * mScaleFactor, 0, 0), mConfig.mMultiplierForward));
         //mListEndposes.push_back(base::Vector3d(mConfig.mSpeedForward / 2.0, 0, 0));
         // Backward
         poses_zero_rad.push_back(endpose(base::Vector3d(mConfig.mSpeedBackward * mScaleFactor, 0, 0), mConfig.mMultiplierBackward));
         // Lateral
         poses_zero_rad.push_back(endpose(base::Vector3d(0.0, mConfig.mSpeedLateral * mScaleFactor, 0.0), mConfig.mMultiplierLateral));
         poses_zero_rad.push_back(endpose(base::Vector3d(0.0, -mConfig.mSpeedLateral * mScaleFactor, 0.0), mConfig.mMultiplierLateral));
         // Point turn
         poses_zero_rad.push_back(endpose(base::Vector3d(0.0, 0.0, mConfig.mSpeedPointTurn * mScaleFactor), mConfig.mMultiplierPointTurn));
         poses_zero_rad.push_back(endpose(base::Vector3d(0.0, 0.0, -mConfig.mSpeedPointTurn * mScaleFactor), mConfig.mMultiplierPointTurn));
         // Forward + negative turn
         // Calculates end pose driving with full forward and turn speeds.
         double turning_radius = mConfig.mSpeedForward / mConfig.mSpeedTurn;
         base::Vector3d turned_start = Eigen::AngleAxis<double>(-mConfig.mSpeedTurn * mScaleFactor, Eigen::Vector3d::UnitZ()) * 
                base::Vector3d(0.0, turning_radius, 0.0);
         turned_start[1] += turning_radius;
         turned_start[2] = -mConfig.mSpeedTurn * mScaleFactor;
         poses_zero_rad.push_back(endpose(turned_start, mConfig.mMultiplierTurn));
         // Forward + positive turn, just mirror positive turn
         turned_start[1] *= -1;
         turned_start[2] *= -1;
         poses_zero_rad.push_back(endpose(turned_start, mConfig.mMultiplierTurn));
         
         // Creates discrete end poses for all angles.
         mListPrimitives.clear();
         std::vector< std::pair<base::Vector3d, int> >::iterator it = poses_zero_rad.begin();
         base::Vector3d vec_tmp;
         Vector3i discrete_pose_tmp;
         double turn_step_rad = (M_PI*2.0) / (double)mConfig.mNumAngles; 
         assert(mConfig.mNumAngles != 0);
         for(unsigned int angle=0; angle<mConfig.mNumAngles; ++angle) {
             for(int id=0; it != poses_zero_rad.end(); ++it, ++id) {
                vec_tmp = Eigen::AngleAxis<double>(angle * turn_step_rad, Eigen::Vector3d::UnitZ()) * it->first;
                // Adapt z component. SBPL orientation uses the range [0, 2*M_PI).
                // TODO correct?
                vec_tmp[2] = vec_tmp[2] + angle * turn_step_rad;
                while(vec_tmp[2] >= 2*M_PI) {
                    vec_tmp[2] -= 2*M_PI;
                }
                // TODO How to round correctly?
                discrete_pose_tmp[0] = (int)(vec_tmp[0] / mConfig.mGridSize);
                discrete_pose_tmp[1] = (int)(vec_tmp[1] / mConfig.mGridSize);
                discrete_pose_tmp[2] = (int)(vec_tmp[2] / turn_step_rad);
                // Scale factor should prevent unwanted (same state) values?
                assert(discrete_pose_tmp[0] != 0);
                assert(discrete_pose_tmp[1] != 0);
                assert((unsigned int)discrete_pose_tmp[2] != angle);
                mListPrimitives.push_back(Primitive(id, angle, discrete_pose_tmp, it->second));
             }
         }
     }
     
     void createIntermediatePoses() {
     }
};

} // end namespace motion_planning_libraries

#endif