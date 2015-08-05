#ifndef _MOTION_PLANNING_LIBRARIES_SBPL_MOTION_PRIMITIVES_HPP_
#define _MOTION_PLANNING_LIBRARIES_SBPL_MOTION_PRIMITIVES_HPP_

#include <fstream>
#include <iomanip> // std::setprecision
#include <iostream>
#include <vector>
#include <map>

#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>

#include <motion_planning_libraries/Config.hpp>

namespace motion_planning_libraries {
    
struct PrimInfo {
    unsigned int mId;
    enum MovementType mMovType;
    double mSpeed;
    
    Id2PrimInfo() : mId(0), mMovType(MOV_UNDEFINED), mSpeed(0.0) {
    }
    
    Id2PrimInfo(unsigned int id, enum MovementType mov_type, double speed) : 
        mId(id), mMovType(mov_type), mSpeed(speed) {
    }
};
    
/**
 * Uses to check for discrete end poses copies and 
 * to avoid c++11 (tuples).
 */
struct Triple {
 public:
    int a,b,c;
    
    Triple() : a(0), b(0), c(0) {}
    Triple(int a_, int b_, int c_) : a(a_), b(b_), c(c_) {}
    
    bool operator==(const Triple& triple) const {
        return this->a == triple.a && this->b == triple.b && this->c == triple.c;
    }
    
    bool operator!=(const Triple& triple) const {
        return !operator==(triple);
    }
    
    bool operator<(const Triple& triple) const {
        if(this->a < triple.a) 
            return true;
        if(this->a == triple.a) {
            if(this->b < triple.b) 
                return true;
            if(this->b == triple.b) {
                if(this->c < triple.c) 
                    return true;
            }
        }
        return false;
    }
};

/**
 * Speed in m/sec or rad/sec.
 */
struct MotionPrimitivesConfig {
    MotionPrimitivesConfig() :
            mMobility(),
            mNumPrimPartition(4),
            mNumPosesPerPrim(2),
            mNumAngles(0),
            mMapWidth(0),
            mMapHeight(0),
            mGridSize(0.0) {   
    }
    
    MotionPrimitivesConfig(Config config, int trav_map_width, int trav_map_height, double grid_size) :
        mMobility(config.mMobility),
        mNumPrimPartition(config.mNumPrimPartition),
        mNumPosesPerPrim(config.mNumIntermediatePoints + 2), // intermediate points + start pose + end pose
        mNumAngles(16),
        mMapWidth(trav_map_width),
        mMapHeight(trav_map_height),
        mGridSize(grid_size),
        mPrimAccuracy(config.mPrimAccuracy) {   
    }   
    
  public:
    struct Mobility mMobility;
    double mNumPrimPartition;
    
    unsigned int mNumPosesPerPrim; // Number of points a primitive consists of.
    unsigned int mNumAngles; // Number of discrete angles (in general 2*M_PI / 16)
    
    unsigned int mMapWidth;
    unsigned int mMapHeight;
    double mGridSize; // Width/length of a grid cell in meter.
    double mPrimAccuracy;
};

/**
 * Describes a single motion primitive. This structure is used
 * to collect the non discrete primitived for angle 0
 * and the final discrete primitives for all the angles.
 */
struct Primitive {
 public:
    unsigned int mId;
    unsigned int mStartAngle;
    /// Can receive non discrete and discrete endposes.
    base::Vector3d mEndPose;
    unsigned int mCostMultiplier;
    std::vector<base::Vector3d> mIntermediatePoses; 
    enum MovementType mMovType; // Type of movement.
    // Will be used to calculate the orientation of the intermediate poses.
    // This orientation is not truncated to 0 to 15.
    int mDiscreteEndOrientationNotTruncated;
    // Stores the center of rotation for curves.
    // Used to calculate the intermediate poses.
    base::Vector3d mCenterOfRotation;
    double mSpeed;
     
    Primitive() : mId(0), mStartAngle(0), mEndPose(), 
            mCostMultiplier(0), mIntermediatePoses(), mMovType(MOV_UNDEFINED), 
            mDiscreteEndOrientationNotTruncated(0), mCenterOfRotation(), mSpeed(0.0)
    {
        mEndPose.setZero();
        mCenterOfRotation.setZero();
    }
    
    /**
     * \param id Id of the motion primitive, is unique for each angle.
     * \param start_angle Discrete starting angle.
     * \param end_pose End pose with <x,y,theta>.
     * \param cost_multiplier Cost multiplier of this kind of motion.
     */
    Primitive(unsigned int id, unsigned int start_angle, base::Vector3d end_pose, 
            unsigned int cost_multiplier, enum MovementType mov_type, double speed) : 
            mId(id), mStartAngle(start_angle), mEndPose(end_pose), 
            mCostMultiplier(cost_multiplier), 
            mIntermediatePoses(), mMovType(mov_type),
            mDiscreteEndOrientationNotTruncated(0), mCenterOfRotation(), mSpeed(speed)
    {
        mCenterOfRotation.setZero();
    }
    
    bool operator==(Primitive& prim) {
        return (this->mStartAngle == prim.mStartAngle && this->mEndPose == prim.mEndPose);
    }
    
    bool operator!=(Primitive& prim) {
        return (this->mStartAngle != prim.mStartAngle || this->mEndPose != prim.mEndPose);
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
 *  - mNumPosesPerPrim: intermediate poses plus start and end with (x_m, y_m, theta_rad)
 */
struct SbplMotionPrimitives {
 public:
    struct MotionPrimitivesConfig mConfig;
    std::vector<struct Primitive> mListPrimitivesAngle0; // Contains non discrete primitives for angle 0.
    std::vector<struct Primitive> mListPrimitives;
    double mRadPerDiscreteAngle;
    std::vector<struct PrimInfo> mId2PrimInfo;
     
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
    /*
    bool createCurvePrimForAngle0(double const forward_speed, double const turning_speed, 
        int const prim_id, int const multiplier, Primitive& primitive);
    */
    
    bool createCurvePrimForAngle0(double const turning_radius_discrete, 
        double const angle_rad_discrete, 
        int const prim_id, 
        int const multiplier, 
        Primitive& primitive);
    
    /**
     * Calculates the discrete angle to the passed non-discrete one.
     */
    int calcDiscreteEndOrientation(double yaw_rad);
    
    /**
     * Each prim id has been assigned a speed value.
     * Currently one speed is used, just inverted for backward movements.
     */
    bool getSpeed(unsigned int const prim_id, double& speed);
    
    /**
     * Calculates the center of rotation for the new discretized end position.
     * Transforms the end pose into the start frame and
     * calculates the intersection of the orthogonal line of the end pose
     * with the y-axis. This takes care that we reach the end position
     * with the correct discrete angle.
     */
    bool calculateOrthogonalIntersection(
        base::Vector3d start_position, double start_theta_rad, 
        base::Vector3d end_position, double end_theta_rad,
        base::Vector3d& cof_grids);
    
    /**
     * Returns the movement type and speed of the passed primitive id.
     */
    bool getPrimInfo(unsigned int id, struct PrimInfo& info) {

        if(id >= mId2PrimInfo.size()) {
            LOG_WARN("Only %d primitive types are available", mId2PrimInfo.size());
            return false;
        }
        
        info = mId2PrimInfo[id];
        return true;
    }
};

} // end namespace motion_planning_libraries

#endif
