#ifndef _MOTION_PLANNING_LIBRARIES_SBPL_MOTION_PRIMITIVES_HPP_
#define _MOTION_PLANNING_LIBRARIES_SBPL_MOTION_PRIMITIVES_HPP_

#include <fstream>
#include <iomanip> // std::setprecision
#include <iostream>
#include <vector>

#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <motion_planning_libraries/Config.hpp>

namespace motion_planning_libraries {

/**
 * Speed in m/sec or rad/sec.
 */
struct MotionPrimitivesConfig {
    MotionPrimitivesConfig() :
            mSpeeds(),
            mNumPrimPartition(4),
            mNumIntermediatePoses(0),
            mNumAngles(0),
            mMapWidth(0),
            mMapHeight(0),
            mGridSize(0.0) {   
    }
    
    MotionPrimitivesConfig(Config config, int trav_map_width, int trav_map_height, double grid_size) :
        mSpeeds(config.mSpeeds),
        mNumPrimPartition(config.mNumPrimPartition),
        mNumIntermediatePoses(10),
        mNumAngles(16),
        mMapWidth(trav_map_width),
        mMapHeight(trav_map_height),
        mGridSize(grid_size) {   
    }   
    
  public:
    struct Speeds mSpeeds;
    double mNumPrimPartition;
    
    unsigned int mNumIntermediatePoses; // Number of points a primitive consists of.
    unsigned int mNumAngles; // Number of discrete angles (in general 2*M_PI / 16)
    
    unsigned int mMapWidth;
    unsigned int mMapHeight;
    double mGridSize; // Width/length of a grid cell in meter.
};

/**
 * Describes a single motion primitive. This structure is used
 * to collect the non discrete primitived for angle 0
 * and the final discrete primitives for all the angles.
 */
struct Primitive {
 public:
    int mId;
    int mStartAngle;
    /// Can receive non discrete and discrete endposes.
    base::Vector3d mEndPose;
    unsigned int mCostMultiplier;
    std::vector<base::Vector3d> mIntermediatePoses; 
    enum MovementType mMovType; // Type of movement.
    // Will be used to calculate the orientation of the intermediate poses.
    // This orientation is not truncated to 0 to 15.
    int mDiscreteEndOrientationNotTruncated;
    // Stores the center of rotation for curves (non discrete). 
    // Used to calculate the intermediate poses.
    base::Vector3d mCenterOfRotationLocal;
     
    Primitive() : mId(0), mStartAngle(0), mEndPose(), 
            mCostMultiplier(0), mIntermediatePoses(), mMovType(MOV_UNDEFINED), 
            mDiscreteEndOrientationNotTruncated(0), mCenterOfRotationLocal()
    {
        mEndPose.setZero();
        mCenterOfRotationLocal.setZero();
    }
    
    /**
     * \param id Id of the motion primitive, is unique for each angle.
     * \param start_angle Discrete starting angle.
     * \param end_pose End pose with <x,y,theta>.
     * \param cost_multiplier Cost multiplier of this kind of motion.
     */
    Primitive(int id, int start_angle, base::Vector3d end_pose, 
            unsigned int cost_multiplier, enum MovementType mov_type) : 
            mId(id), mStartAngle(start_angle), mEndPose(end_pose), 
            mCostMultiplier(cost_multiplier), 
            mIntermediatePoses(), mMovType(mov_type),
            mDiscreteEndOrientationNotTruncated(0), mCenterOfRotationLocal()
    {
        mCenterOfRotationLocal.setZero();
    }
    
    /** 
     * Stores the passed discrete orientation with truncated orientation (0 to mNumAngles) to
     * mEndPose and stores the not-truncated orientation to 
     * mDiscreteEndOrientationNotTruncated.
     * The non truncated value will be used to calculate the orientation of the
     * intermediate poses.
     * \param discrete_theta Discrete orientation. One discrete angle 
     * represents 2*M_PI/num_angles radians.
     * \param num_angles Number of discrete angles in SBPL, default: [0,16)
     */
    void setDiscreteEndOrientation(int discrete_theta, int num_angles) {
        mDiscreteEndOrientationNotTruncated = discrete_theta;
        while (discrete_theta >= num_angles)
            discrete_theta -= num_angles;
        while (discrete_theta < 0)
            discrete_theta += num_angles;
        mEndPose[2] = discrete_theta;
    }
        
    int getDiscreteEndOrientationNotTruncated() {
        return mDiscreteEndOrientationNotTruncated;
    }
    
    std::string toString() {
        std::stringstream ss;
        ss << "starting angle: " << mStartAngle << ", id: " << mId << ", movement type: " << 
                (int)mMovType << ", endpose: " << mEndPose.transpose();
        return ss.str();
    }
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
struct SbplMotionPrimitives {
 public:
    struct MotionPrimitivesConfig mConfig;
    std::vector<struct Primitive> mListPrimitivesAngle0; // Contains non discrete primitives for angle 0.
    std::vector<struct Primitive> mListPrimitives;
    // Scale factor used to scale all primitives to take sure that they reach a new state.
    double mScaleFactor; 
    double mRadPerDiscreteAngle;
     
    SbplMotionPrimitives();
     
    SbplMotionPrimitives(struct MotionPrimitivesConfig config);
    
    ~SbplMotionPrimitives();
    
    /**
     * Fills mListPrimitives.
     */
    void createPrimitives();
    
    /**
     * Uses the defined speeds of the system to create the motion primitives 
     * for discrete angle 0.
     */
    std::vector<struct Primitive> createMPrimsForAngle0();

    /**
     * Uses the passed list of angle 0 non discrete motion primitives to
     * calculate all primitives. This is done by rotating the angle 0 prims
     * mNumAngles-1 times to cover the complete 2*M_PI and to find the discrete
     * pose.
     */
    std::vector<struct Primitive> createMPrims(std::vector<struct Primitive> prims_angle_0);
    
    /**
     * Runs through all the discrete motion primitives and adds the
     * non discrete intermediate poses. This is done with the non truncated
     * end orientation stored within the primitive structure.
     */
    void createIntermediatePoses(std::vector<struct Primitive>& discrete_mprims);
    
    void storeToFile(std::string path);
    
    /**
     * Forward and turning speed does already contain the scale factor.
     * Uses grid_local.
     */
    Primitive createCurvePrimForAngle0(double forward_speed, double turning_speed, 
            int prim_id, int multiplier);
};

} // end namespace motion_planning_libraries

#endif
