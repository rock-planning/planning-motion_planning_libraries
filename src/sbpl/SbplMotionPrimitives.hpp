#ifndef _MOTION_PLANNING_LIBRARIES_SBPL_MOTION_PRIMITIVES_HPP_
#define _MOTION_PLANNING_LIBRARIES_SBPL_MOTION_PRIMITIVES_HPP_

#include <fstream>
#include <iomanip> // std::setprecision
#include <iostream>
#include <vector>

#include <base/Eigen.hpp>

#include <motion_planning_libraries/Config.hpp>

namespace motion_planning_libraries {

/**
 * Speed in m/sec or rad/sec.
 */
struct MotionPrimitivesConfig {
    MotionPrimitivesConfig() :
            mSpeeds(),
            mNumTurnPrimitives(0),
            mNumIntermediatePoses(0),
            mNumAngles(0),
            mMapWidth(0),
            mMapHeight(0),
            mGridSize(0.0) {   
    }
    
    MotionPrimitivesConfig(Config config, int trav_map_width, int trav_map_height, double grid_size) :
        mSpeeds(config.mSpeeds),
        mNumTurnPrimitives(2),
        mNumIntermediatePoses(10),
        mNumAngles(16),
        mMapWidth(trav_map_width),
        mMapHeight(trav_map_height),
        mGridSize(grid_size) {   
    }   
    
  public:
    struct Speeds mSpeeds;
    unsigned int mNumTurnPrimitives;
    
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
     
    Primitive() : mId(0), mStartAngle(0), mEndPose(), 
            mCostMultiplier(0), mIntermediatePoses(), mMovType(MOV_UNDEFINED)
    {
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
            mIntermediatePoses(), mMovType(mov_type)
    {
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
     
    SbplMotionPrimitives() : mConfig(), 
            mListPrimitives(), mScaleFactor(0), mRadPerDiscreteAngle(0) {
    }
     
    SbplMotionPrimitives(struct MotionPrimitivesConfig config) : mConfig(config),
            mListPrimitives(), mScaleFactor(0), mRadPerDiscreteAngle(0)
    {
        mRadPerDiscreteAngle = (M_PI*2.0) / (double)mConfig.mNumAngles;
    }
    
    ~SbplMotionPrimitives() {
    }
    
    /**
     * Fills mListPrimitives.
     */
    void createPrimitives() {
        std::vector<struct Primitive> prim_angle_0 = createMPrimsForAngle0();
        createMPrims(prim_angle_0); // Stores to global prim list mListPrimitives as well.
        createIntermediatePoses(mListPrimitives); // Adds intermediate poses.
    }
    
    /**
     * Uses the defined speeds of the system to create the motion primitives 
     * for discrete angle 0.
     */
    std::vector<struct Primitive> createMPrimsForAngle0() { 
        mListPrimitivesAngle0.clear();
        
        // Scaling: Each movement has to reach a new discrete state, so we have
        // to find the min scale factor. This represents the required movement time
        // to create all these primitives.
        mScaleFactor = 1.0;
        if(mConfig.mSpeeds.mSpeedForward > 0) {
            mScaleFactor = std::max(mScaleFactor, mConfig.mGridSize / mConfig.mSpeeds.mSpeedForward);
            LOG_INFO("Speed forward, new scale factor: %4.2f", mScaleFactor);
        }
        if(mConfig.mSpeeds.mSpeedBackward > 0) {
            mScaleFactor = std::max(mScaleFactor, mConfig.mGridSize / mConfig.mSpeeds.mSpeedBackward);
            LOG_INFO("Speed backward, new scale factor: %4.2f", mScaleFactor);
        }
        if(mConfig.mSpeeds.mSpeedLateral > 0) {
            mScaleFactor = std::max(mScaleFactor, mConfig.mGridSize / mConfig.mSpeeds.mSpeedLateral);
            LOG_INFO("Speed lateral, new scale factor: %4.2f", mScaleFactor);
        }
        if(mConfig.mSpeeds.mSpeedPointTurn > 0) {
            mScaleFactor = std::max(mScaleFactor, mRadPerDiscreteAngle / mConfig.mSpeeds.mSpeedPointTurn);
            LOG_INFO("Speed point turn, new scale factor: %4.2f", mScaleFactor);
        }
        if(mConfig.mSpeeds.mSpeedTurn > 0) {
            mScaleFactor = std::max(mScaleFactor, mRadPerDiscreteAngle / mConfig.mSpeeds.mSpeedTurn);
            LOG_INFO("Speed turn, new scale factor: %4.2f", mScaleFactor);
        }
        LOG_INFO("Overall primitives scale factor (movement time in seconds): %4.2f", mScaleFactor);
        
        int primId = 0;
        // Forward
        if(mConfig.mSpeeds.mSpeedForward > 0) {
            // TODO: Not optimal I guess: mConfig.mGridSize * 2 is used (independently of the scale factor)
            // to improve goal-reaching. Grid size is multiplied by two to take sure that
            // new x and y values are reached for all 16 angles.
            mListPrimitivesAngle0.push_back( Primitive(primId++, 
                    0, 
                    base::Vector3d(mConfig.mGridSize * 2, 0.0, 0.0),
                    mConfig.mSpeeds.mMultiplierForward, 
                    MOV_FORWARD));
            mListPrimitivesAngle0.push_back( Primitive(primId++, 
                    0, 
                    base::Vector3d(mConfig.mSpeeds.mSpeedForward * mScaleFactor, 0.0, 0.0),
                    mConfig.mSpeeds.mMultiplierForward, 
                    MOV_FORWARD));
        }
        // Backward
        if(mConfig.mSpeeds.mSpeedBackward > 0) {
            mListPrimitivesAngle0.push_back( Primitive(primId++,
                    0,
                    base::Vector3d(-mConfig.mSpeeds.mSpeedBackward * mScaleFactor, 0.0, 0.0),
                    mConfig.mSpeeds.mMultiplierBackward, 
                    MOV_BACKWARD));
        }
        // Lateral
        if(mConfig.mSpeeds.mSpeedLateral > 0) {
            mListPrimitivesAngle0.push_back( Primitive(primId++, 
                    0,
                    base::Vector3d(0.0, mConfig.mSpeeds.mSpeedLateral * mScaleFactor, 0.0),
                    mConfig.mSpeeds.mMultiplierLateral,
                    MOV_LATERAL));
            mListPrimitivesAngle0.push_back( Primitive(primId++, 
                    0,
                    base::Vector3d(0.0, -mConfig.mSpeeds.mSpeedLateral * mScaleFactor, 0.0),
                    mConfig.mSpeeds.mMultiplierLateral, 
                    MOV_LATERAL));
        }
        // Point turn
        if(mConfig.mSpeeds.mSpeedPointTurn > 0) {
            mListPrimitivesAngle0.push_back( Primitive(primId++,
                    0,
                    base::Vector3d(0.0, 0.0, mConfig.mSpeeds.mSpeedPointTurn * mScaleFactor),
                    mConfig.mSpeeds.mMultiplierPointTurn,
                    MOV_POINTTURN));
            mListPrimitivesAngle0.push_back( Primitive(primId++, 
                    0,
                    base::Vector3d(0.0, 0.0, -mConfig.mSpeeds.mSpeedPointTurn * mScaleFactor),
                    mConfig.mSpeeds.mMultiplierPointTurn,
                    MOV_POINTTURN));
        }
        // Forward + negative turn
        // Calculates end pose driving with full forward and turn speeds.
        if(mConfig.mSpeeds.mSpeedForward > 0 && mConfig.mSpeeds.mSpeedTurn > 0) {
            // This way of calculating the turning radius seems to be inaccurate.
            double turning_radius = mConfig.mSpeeds.mSpeedForward / mConfig.mSpeeds.mSpeedTurn;
            // Vector3d: x y theta
            base::Vector3d turned_start = Eigen::AngleAxis<double>(-mConfig.mSpeeds.mSpeedTurn * mScaleFactor, Eigen::Vector3d::UnitZ()) * 
                    base::Vector3d(0.0, turning_radius, 0.0);
            turned_start[1] -= turning_radius;
            turned_start[2] = -mConfig.mSpeeds.mSpeedTurn * mScaleFactor;
            mListPrimitivesAngle0.push_back( Primitive(primId++, 
                    0,
                    turned_start,  
                    mConfig.mSpeeds.mMultiplierTurn, 
                    MOV_TURN));
            // Forward + positive turn, just mirror negative turn
            turned_start[1] *= -1;
            turned_start[2] *= -1;
            mListPrimitivesAngle0.push_back( Primitive(primId++, 
                    0,
                    turned_start, 
                    mConfig.mSpeeds.mMultiplierTurn, 
                    MOV_TURN));
        }
        
        return mListPrimitivesAngle0;
    }

    /**
     * Uses the passed list of angle 0 non discrete motion primitives to
     * calculate all primitives. This is done by rotating the angle 0 prims
     * mNumAngles-1 times to cover the complete 2*M_PI and to find the discrete
     * pose.
     */
    std::vector<struct Primitive> createMPrims(std::vector<struct Primitive> prims_angle_0) {

        // Creates discrete end poses for all angles.
        mListPrimitives.clear();
        base::Vector3d vec_tmp;
        base::Vector3d discrete_endpose_tmp;
        double theta_tmp = 0.0;
        assert(mConfig.mNumAngles != 0);
        // Runs through all discrete angles (default 16)
        for(unsigned int angle=0; angle < mConfig.mNumAngles; ++angle) {
            std::vector< struct Primitive >::iterator it = prims_angle_0.begin();
            
            // Runs through all endposes in the world which have been defined for angle 0.
            for(int id=0; it != prims_angle_0.end(); ++it, ++id) {
                // Extract x,y,theta from the Vector3d.
                vec_tmp = it->mEndPose;
                theta_tmp = vec_tmp[2];
                vec_tmp[2] = 0;
                vec_tmp = Eigen::AngleAxis<double>(angle * mRadPerDiscreteAngle, Eigen::Vector3d::UnitZ()) * vec_tmp;
                
                // Calculates discrete pose.
                // TODO How to round correctly? How much inaccuracy is acceptable?
                discrete_endpose_tmp[0] = (int)(round(vec_tmp[0] / mConfig.mGridSize));
                discrete_endpose_tmp[1] = (int)(round(vec_tmp[1] / mConfig.mGridSize));
                        
                // SBPL orientation uses the range [0, 2*M_PI), intermediate points should get (-PI,PI].
                theta_tmp = theta_tmp + angle * mRadPerDiscreteAngle;
                // Discrete value can be < 0 and > mNumAngles. Will be stored for intermediate point calculation.
                discrete_endpose_tmp[2] = ((int)round(theta_tmp / mRadPerDiscreteAngle));
                
                // We have to reach another discrete state. So regarding to the
                // movement type we have to reach another discrete grid coordinate
                // or another discrete angle.
                bool new_state = true;
                int original_x_discrete = (int)(round(it->mEndPose[0] / mConfig.mGridSize));
                int original_y_discrete = (int)(round(it->mEndPose[1] / mConfig.mGridSize));
                int original_theta_discrete = (int)(round(it->mEndPose[2] / mRadPerDiscreteAngle));
                switch(it->mMovType) {
                    case MOV_POINTTURN: {
                        if(discrete_endpose_tmp[2] == original_theta_discrete) {
                            new_state = false;
                        }
                        break;
                    } 
                    case MOV_TURN: {
                        // TODO Correct?
                        if((discrete_endpose_tmp[0] == original_x_discrete &&
                            discrete_endpose_tmp[1] == original_y_discrete) ||
                            discrete_endpose_tmp[2] == original_theta_discrete) {
                            new_state = false;
                        }
                        break;
                    }
                    default: {
                        if(discrete_endpose_tmp[0] == original_x_discrete &&
                            discrete_endpose_tmp[1] == original_y_discrete) {
                            new_state = false;
                        }
                        break;
                    }
                }
                if(!new_state) {
                    LOG_WARN("Primitive %d of type %d for angle %d does not lead into a new state", 
                            id, (int)it->mMovType, angle);
                }
                
                Primitive prim_discrete(it->mId, angle, discrete_endpose_tmp, it->mCostMultiplier, it->mMovType);
                // The orientation of the discrete endpose still can exceed the borders 0 to mNumAngles.
                // We will store this for the intermediate point calculation, but the orientation
                // of the discrete end pose will be truncated to [0,mNumAngles).
                prim_discrete.setDiscreteEndOrientation(discrete_endpose_tmp[2], mConfig.mNumAngles);
                mListPrimitives.push_back(prim_discrete);
            }
        }
        return mListPrimitives;
    }
    
    /**
     * Runs through all the discrete motion primitives and adds the
     * non discrete intermediate poses. This is done with the non truncated
     * end orientation stored within the primitive structure.
     */
    void createIntermediatePoses(std::vector<struct Primitive>& discrete_mprims) {

        std::vector<struct Primitive>::iterator it = discrete_mprims.begin();
        base::Vector3d end_pose_world;
        base::Vector3d intermediate_pose;
        double x_step=0.0, y_step=0.0, theta_step=0.0;
        int discrete_rot_diff = 0;
        double start_orientation_world = 0.0;
        
        for(;it != discrete_mprims.end(); it++) {
            start_orientation_world = it->mStartAngle * mRadPerDiscreteAngle;
        
            // Theta range is 0 to 15, have to be sure to use the shortest rotation.
            // And of course the starting orientation has to be regarded!
            
            discrete_rot_diff = it->getDiscreteEndOrientationNotTruncated() - it->mStartAngle;
            /*
            if(discrete_rot_diff > (int)(mConfig.mNumAngles / 2)) {
                discrete_rot_diff -= mConfig.mNumAngles;
            }
            */
            
            
            end_pose_world[0] = it->mEndPose[0] * mConfig.mGridSize;
            end_pose_world[1] = it->mEndPose[1] * mConfig.mGridSize;
            
            x_step = end_pose_world[0] / ((double)mConfig.mNumIntermediatePoses-1);
            y_step = end_pose_world[1] / ((double)mConfig.mNumIntermediatePoses-1);
            theta_step = (discrete_rot_diff * mRadPerDiscreteAngle) / ((double)mConfig.mNumIntermediatePoses-1);
            
            for(unsigned int i=0; i<mConfig.mNumIntermediatePoses; i++) {
                intermediate_pose[0] = i * x_step;
                intermediate_pose[1] = i * y_step;
                // Forward, backward or lateral movement, orientation does not change.
                if(it->mStartAngle == it->mEndPose[2]) {
                    intermediate_pose[2] = start_orientation_world;
                } else {
                    intermediate_pose[2] = start_orientation_world + i * theta_step;
                }
                // Truncate orientation of intermediate poses to (-PI,PI] (according to OMPL).
                while(intermediate_pose[2] <= -M_PI)
                    intermediate_pose[2] += 2*M_PI;
                while(intermediate_pose[2] > M_PI)
                    intermediate_pose[2] -= 2*M_PI;
                it->mIntermediatePoses.push_back(intermediate_pose);
            }
            //it->mIntermediatePoses.push_back(end_pose_world);
        }
    }
    
    void storeToFile(std::string path) {
        std::ofstream mprim_file;
        mprim_file.open(path.c_str());
        
        mprim_file << "resolution_m: " <<  std::fixed << std::setprecision(6) << mConfig.mGridSize << std::endl;
        mprim_file << "numberofangles: " << mConfig.mNumAngles << std::endl;
        mprim_file << "totalnumberofprimitives: " << mListPrimitives.size() << std::endl;
        
        
        struct Primitive mprim;
        for(unsigned int i=0; i < mListPrimitives.size(); ++i) {
            mprim =  mListPrimitives[i];
            mprim_file << "primID: " << mprim.mId << std::endl;
            mprim_file << "startangle_c: " << mprim.mStartAngle << std::endl;
            mprim_file << "endpose_c: " << (int)mprim.mEndPose[0] << " " << 
                    (int)mprim.mEndPose[1] << " " << (int)mprim.mEndPose[2] << std::endl;
            mprim_file << "additionalactioncostmult: " << mprim.mCostMultiplier << std::endl;
            mprim_file << "intermediateposes: " << mprim.mIntermediatePoses.size() << std::endl;
            base::Vector3d v3d;
            for(unsigned int i=0; i < mprim.mIntermediatePoses.size(); ++i) {
                v3d = mprim.mIntermediatePoses[i];
                mprim_file << std::fixed << std::setprecision(4) << 
                        v3d[0] << " " << v3d[1] << " " << v3d[2] << std::endl;
            }
        }
        
        mprim_file.close();
    }
};

} // end namespace motion_planning_libraries

#endif
