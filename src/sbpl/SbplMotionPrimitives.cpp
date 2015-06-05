#include "SbplMotionPrimitives.hpp"

namespace motion_planning_libraries {

SbplMotionPrimitives::SbplMotionPrimitives() : mConfig(), 
        mListPrimitives(), mRadPerDiscreteAngle(0), mMapPrimID2Speeds() 
{
}
    
SbplMotionPrimitives::SbplMotionPrimitives(struct MotionPrimitivesConfig config) : mConfig(config),
        mListPrimitives(), mRadPerDiscreteAngle(0), mMapPrimID2Speeds()
{
    mRadPerDiscreteAngle = (M_PI*2.0) / (double)mConfig.mNumAngles;
}

SbplMotionPrimitives::~SbplMotionPrimitives() {
}

/**
    * Fills mListPrimitives.
    */
void SbplMotionPrimitives::createPrimitives() {  
    // TODO: Fix/check this!
    if(mConfig.mNumPrimPartition != 1 &&
        mConfig.mNumPrimPartition != 2 &&
        mConfig.mNumPrimPartition != 4 &&
        mConfig.mNumPrimPartition != 8) {
        LOG_WARN("Currently only 1, 2, 4 or 8 are valid for mNumPrimPartition!");
    }
    
    std::vector<struct Primitive> prim_angle_0 = createMPrimsForAngle0();
    createMPrims(prim_angle_0); // Stores to global prim list mListPrimitives as well.
    createIntermediatePoses(mListPrimitives); // Adds intermediate poses.
}

/**
    * Uses the defined speeds of the system to create the motion primitives 
    * for discrete angle 0.
    */
std::vector<struct Primitive> SbplMotionPrimitives::createMPrimsForAngle0() { 
    mListPrimitivesAngle0.clear();
    mMapPrimID2Speeds.clear();
        
    // mConfig.mGridSize * 2 is used (independently of the scale factor)
    // to improve new-discrete-state-reaching. Grid size is multiplied by two 
    // to take sure that new x and y values are reached for all 16 angles.
    double min_prim_length = mConfig.mGridSize * 2;
    int primId = 0;
    Primitive prim;
    
    assert (mConfig.mNumPrimPartition >= 1);
    
    // All prims of one type will receive their defined max speed regardless  of their length.
    // Only for the curves it is required to use the correct speeds.
    
    // Forward
    for(double i=1; i < mConfig.mNumPrimPartition+1; i++) {
        if(mConfig.mSpeeds.mSpeedForward > 0) {
            double scale_factor_forward = std::max(1.0, min_prim_length / 
                    (mConfig.mSpeeds.mSpeedForward / mConfig.mNumPrimPartition));
            LOG_INFO("Speed forward scale factor: %4.2f", scale_factor_forward);
            
            prim = Primitive(primId, 
                    0, 
                    base::Vector3d((mConfig.mSpeeds.mSpeedForward / i) * scale_factor_forward, 0.0, 0.0),
                    mConfig.mSpeeds.mMultiplierForward, 
                    MOV_FORWARD);
            prim.mSpeeds.mSpeedForward = mConfig.mSpeeds.mSpeedForward;

            mListPrimitivesAngle0.push_back(prim);
            mMapPrimID2Speeds.push_back(prim.mSpeeds);
            primId++;
        }
    }
    
    // Backward
    for(double i=1; i < mConfig.mNumPrimPartition+1; i++) {   
        if(mConfig.mSpeeds.mSpeedBackward < 0) {
            LOG_WARN("Backward speed has to be positive, no backward movement will be available!");
        }
        if(mConfig.mSpeeds.mSpeedBackward > 0) {
            double scale_factor_backward = std::max(1.0, min_prim_length / 
                    (mConfig.mSpeeds.mSpeedBackward / mConfig.mNumPrimPartition));
            LOG_INFO("Speed backward scale factor: %4.2f", scale_factor_backward);
            
            prim = Primitive(primId,
                    0,
                    base::Vector3d((-mConfig.mSpeeds.mSpeedBackward / i) * scale_factor_backward, 0.0, 0.0),
                    mConfig.mSpeeds.mMultiplierBackward, 
                    MOV_BACKWARD);
            prim.mSpeeds.mSpeedBackward = -mConfig.mSpeeds.mSpeedBackward;

            mListPrimitivesAngle0.push_back(prim);
            mMapPrimID2Speeds.push_back(prim.mSpeeds);
            primId++;
        }   
    }
    
    // Lateral
    for(double i=1; i < mConfig.mNumPrimPartition+1; i++) { 
        if(mConfig.mSpeeds.mSpeedLateral > 0) {
            double scale_factor_lateral = std::max(1.0, min_prim_length / 
                    (mConfig.mSpeeds.mSpeedLateral / mConfig.mNumPrimPartition));
            LOG_INFO("Speed lateral scale factor: %4.2f", scale_factor_lateral);
            
            prim = Primitive(primId, 
                    0,
                    base::Vector3d(0.0, (mConfig.mSpeeds.mSpeedLateral / i) * scale_factor_lateral, 0.0),
                    mConfig.mSpeeds.mMultiplierLateral,
                    MOV_LATERAL);
            prim.mSpeeds.mSpeedLateral = mConfig.mSpeeds.mSpeedLateral;
            mListPrimitivesAngle0.push_back(prim);
            mMapPrimID2Speeds.push_back(prim.mSpeeds);
            primId++;
            
            prim = Primitive(primId, 
                    0,
                    base::Vector3d(0.0, (-mConfig.mSpeeds.mSpeedLateral / i) * scale_factor_lateral, 0.0),
                    mConfig.mSpeeds.mMultiplierLateral, 
                    MOV_LATERAL);
            prim.mSpeeds.mSpeedLateral = -mConfig.mSpeeds.mSpeedLateral;
            mListPrimitivesAngle0.push_back(prim);
            mMapPrimID2Speeds.push_back(prim.mSpeeds);
            primId++;
        }
    }
    
    // Point turn
    for(double i=1; i < mConfig.mNumPrimPartition+1; i++) { 
        if(mConfig.mSpeeds.mSpeedPointTurn > 0) {
            double scale_factor_pointturn = std::max(1.0, mRadPerDiscreteAngle / 
                    (mConfig.mSpeeds.mSpeedPointTurn / mConfig.mNumPrimPartition));
            LOG_INFO("Speed point turn scale factor: %4.2f", scale_factor_pointturn);
            
            prim = Primitive(primId,
                    0,
                    base::Vector3d(0.0, 0.0, (mConfig.mSpeeds.mSpeedPointTurn / i) * scale_factor_pointturn),
                    mConfig.mSpeeds.mMultiplierPointTurn,
                    MOV_POINTTURN);
            prim.mSpeeds.mSpeedPointTurn = mConfig.mSpeeds.mSpeedPointTurn;
            mListPrimitivesAngle0.push_back(prim);
            mMapPrimID2Speeds.push_back(prim.mSpeeds);
            primId++;
            
            prim = Primitive(primId, 
                    0,
                    base::Vector3d(0.0, 0.0, (-mConfig.mSpeeds.mSpeedPointTurn / i) * scale_factor_pointturn),
                    mConfig.mSpeeds.mMultiplierPointTurn,
                    MOV_POINTTURN);
            prim.mSpeeds.mSpeedPointTurn = -mConfig.mSpeeds.mSpeedPointTurn;
            mListPrimitivesAngle0.push_back(prim);
            mMapPrimID2Speeds.push_back(prim.mSpeeds);
            primId++;
        }
        
    }
    
    // Forward turns
    for(double i=1; i < mConfig.mNumPrimPartition+1; i++) { 
        // Create forward and backward curves.
        // First calculates a common scale factor for the minimal forward/backward- and 
        // turning-speed. 
        if(mConfig.mSpeeds.mSpeedForward > 0 && mConfig.mSpeeds.mSpeedTurn > 0) {

            double scale_factor_forward_turn = std::max(1.0, 
                    min_prim_length / (mConfig.mSpeeds.mSpeedForward / mConfig.mNumPrimPartition));
            scale_factor_forward_turn = std::max(scale_factor_forward_turn, 
                    mRadPerDiscreteAngle / (mConfig.mSpeeds.mSpeedTurn / mConfig.mNumPrimPartition));
            LOG_INFO("Speed forward turn scale factor: %4.2f", scale_factor_forward_turn);
            
            double forward_speed = (mConfig.mSpeeds.mSpeedForward / i);
            double forward_turn_speed = (mConfig.mSpeeds.mSpeedTurn / (mConfig.mNumPrimPartition+1 - i));
            
            // Takes care that we keep a minimal speed, so just the turning speed will be increased.
            if(mConfig.mSpeeds.mMinSpeed > 0 && forward_speed < mConfig.mSpeeds.mMinSpeed) {
                forward_speed = mConfig.mSpeeds.mMinSpeed;
            }
            
            // TODO: For turn-multipliers mConfig.mSpeeds.mMultiplierTurn * i has been used.
            // But SBPL already creates higher costs for narrow curves, does it?
            // Create left hand bend.
            if(createCurvePrimForAngle0(forward_speed * scale_factor_forward_turn, 
                        forward_turn_speed * scale_factor_forward_turn, 
                        primId, 
                        mConfig.mSpeeds.mMultiplierTurn,
                        prim)) {
                prim.mSpeeds.mSpeedForward = forward_speed;
                prim.mSpeeds.mSpeedTurn = forward_turn_speed;
                mListPrimitivesAngle0.push_back(prim);
                mMapPrimID2Speeds.push_back(prim.mSpeeds);
                primId++;
            }
            
            // Create right hand bend.
            if(createCurvePrimForAngle0(forward_speed * scale_factor_forward_turn, 
                        -forward_turn_speed * scale_factor_forward_turn, 
                        primId, 
                        mConfig.mSpeeds.mMultiplierTurn, 
                        prim)) {
                prim.mSpeeds.mSpeedForward = forward_speed;
                prim.mSpeeds.mSpeedTurn = -forward_turn_speed;
                mListPrimitivesAngle0.push_back(prim);
                mMapPrimID2Speeds.push_back(prim.mSpeeds);
                primId++;
            }
        }
    }
    
    // Backward turns
    for(double i=1; i < mConfig.mNumPrimPartition+1; i++) { 
        if(mConfig.mSpeeds.mSpeedBackward > 0 && mConfig.mSpeeds.mSpeedTurn > 0) {    
            double scale_factor_backward_turn = std::max(1.0, min_prim_length / 
                    (mConfig.mSpeeds.mSpeedBackward / mConfig.mNumPrimPartition));
            scale_factor_backward_turn = std::max(scale_factor_backward_turn, mRadPerDiscreteAngle / 
                    (mConfig.mSpeeds.mSpeedTurn / mConfig.mNumPrimPartition));
            LOG_INFO("Speed backward turn scale factor: %4.2f", scale_factor_backward_turn);
            
            double backward_speed = (mConfig.mSpeeds.mSpeedBackward / i);
            double backward_turn_speed = (mConfig.mSpeeds.mSpeedTurn / (mConfig.mNumPrimPartition+1 - i));
            
            // Takes care that we keep a minimal speed, so just the turning speed will be increased.
            if(mConfig.mSpeeds.mMinSpeed > 0 && backward_speed < mConfig.mSpeeds.mMinSpeed) {
                backward_speed = mConfig.mSpeeds.mMinSpeed;
            }

            // TODO Using backward multiplier here, actually we need a backward-turn multiplier?
            // Create left hand bend.
            if(createCurvePrimForAngle0(-backward_speed * scale_factor_backward_turn, 
                        -backward_turn_speed * scale_factor_backward_turn, 
                        primId, 
                        mConfig.mSpeeds.mMultiplierBackwardTurn,
                        prim)) {
                prim.mSpeeds.mSpeedBackward = -backward_speed;
                prim.mSpeeds.mSpeedTurn = -backward_turn_speed;
                mListPrimitivesAngle0.push_back(prim);
                mMapPrimID2Speeds.push_back(prim.mSpeeds);
                primId++;
            }
            
            // Create right hand bend.
            if(createCurvePrimForAngle0(-backward_speed * scale_factor_backward_turn, 
                        backward_turn_speed * scale_factor_backward_turn, 
                        primId, 
                        mConfig.mSpeeds.mMultiplierBackwardTurn,
                        prim)) {
                prim.mSpeeds.mSpeedBackward = -backward_speed;
                prim.mSpeeds.mSpeedTurn = backward_turn_speed;
                mListPrimitivesAngle0.push_back(prim); 
                mMapPrimID2Speeds.push_back(prim.mSpeeds);
                primId++;
            }
        }
    }

    return mListPrimitivesAngle0;
}

/**
    * Uses the passed list of angle 0 non discrete motion primitives to
    * calculate all primitives. This is done by rotating the angle 0 prims
    * mNumAngles-1 times to cover the complete 2*M_PI and to find the discrete
    * pose.
    */
std::vector<struct Primitive> SbplMotionPrimitives::createMPrims(std::vector<struct Primitive> prims_angle_0) {

    // Creates discrete end poses for all angles.
    mListPrimitives.clear();
    base::Vector3d vec_tmp;
    base::Vector3d discrete_endpose_tmp;
    base::Vector3d turned_center_of_rotation;
    double theta_tmp = 0.0;
    assert(mConfig.mNumAngles != 0);
    // Runs through all discrete angles (default 16)
    for(unsigned int angle=0; angle < mConfig.mNumAngles; ++angle) {
        std::vector< struct Primitive >::iterator it = prims_angle_0.begin();
        
        // Runs through all endposes in grid-local which have been defined for angle 0.
        for(int id=0; it != prims_angle_0.end(); ++it, ++id) {
            // Extract x,y,theta from the Vector3d.
            vec_tmp = it->mEndPose;
            theta_tmp = vec_tmp[2];
            vec_tmp[2] = 0;
            vec_tmp = Eigen::AngleAxis<double>(angle * mRadPerDiscreteAngle, Eigen::Vector3d::UnitZ()) * vec_tmp;
            
            // Turn center of rotation vector as well
            turned_center_of_rotation = Eigen::AngleAxis<double>(angle * mRadPerDiscreteAngle, Eigen::Vector3d::UnitZ()) * 
                    it->mCenterOfRotationLocal;
            
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
            int original_x_discrete = 0; //(int)(round(it->mEndPose[0] / mConfig.mGridSize));
            int original_y_discrete = 0; //(int)(round(it->mEndPose[1] / mConfig.mGridSize));
            int original_theta_discrete = angle; //(int)(round(it->mEndPose[2] / mRadPerDiscreteAngle));
            
            switch(it->mMovType) {
                case MOV_POINTTURN: {
                    if(discrete_endpose_tmp[2] == original_theta_discrete) {
                        new_state = false;
                    }
                    break;
                }
                case MOV_FORWARD_TURN:
                case MOV_BACKWARD_TURN: {
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
            prim_discrete.mCenterOfRotationLocal = turned_center_of_rotation;
            prim_discrete.mSpeeds = it->mSpeeds;
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
void SbplMotionPrimitives::createIntermediatePoses(std::vector<struct Primitive>& discrete_mprims) {

    std::vector<struct Primitive>::iterator it = discrete_mprims.begin();
    base::Vector3d end_pose_local;
    end_pose_local.setZero();
    base::Vector3d center_of_rotation_local;
    center_of_rotation_local.setZero();
    base::Vector3d intermediate_pose;
    double x_step=0.0, y_step=0.0, theta_step=0.0;
    int discrete_rot_diff = 0;
    double start_orientation_local = 0.0;
    
    // Turn variables.
    base::samples::RigidBodyState rbs_cor;
    base::Vector3d base_local;
    base::Affine3d cor2base;
    base::Affine3d base2cor;
    double len_diff_delta = 0.0;
    double len_scale_factor = 0.0;
    double angle_delta = 0.0;
    double len_base_local = 0.0;
    
    std::stringstream ss;
    
    for(;it != discrete_mprims.end(); it++) {
        ss << std::endl << "Create intermediate poses for prim " << it->toString() << std::endl;
     
        start_orientation_local = it->mStartAngle * mRadPerDiscreteAngle;
    
        // Theta range is 0 to 15, have to be sure to use the shortest rotation.
        // And of course the starting orientation has to be regarded!
        discrete_rot_diff = it->getDiscreteEndOrientationNotTruncated() - it->mStartAngle;
        
        end_pose_local[0] = it->mEndPose[0] * mConfig.mGridSize;
        end_pose_local[1] = it->mEndPose[1] * mConfig.mGridSize;
        
        x_step = end_pose_local[0] / ((double)mConfig.mNumPosesPerPrim-1);
        y_step = end_pose_local[1] / ((double)mConfig.mNumPosesPerPrim-1);
        theta_step = (discrete_rot_diff * mRadPerDiscreteAngle) / ((double)mConfig.mNumPosesPerPrim-1);
        
        if(it->mMovType == MOV_FORWARD_TURN || it->mMovType == MOV_BACKWARD_TURN) {
            // Everything has to be transformed to local, center of rotation as well..
            // TODO: Center of rotation is already in grid_local?
            center_of_rotation_local[0] = it->mCenterOfRotationLocal[0];// * mConfig.mGridSize;
            center_of_rotation_local[1] = it->mCenterOfRotationLocal[1];// * mConfig.mGridSize;
            
            ss << "center of rotation: " << center_of_rotation_local.transpose() << std::endl;
            
            // Create transformation center of rotation to base
            rbs_cor.position = center_of_rotation_local;
            rbs_cor.orientation = Eigen::AngleAxis<double>(start_orientation_local, Eigen::Vector3d::UnitZ());
            cor2base = rbs_cor.getTransform();
            base2cor = cor2base.inverse();
            
            // Transform base (0,0) and endpose_local into the center of rotation frame.
            base_local.setZero();
            base_local = base2cor * base_local;
            end_pose_local = base2cor * end_pose_local;
            
            ss << "base vector: " << base_local.transpose() << ", end vector: " << end_pose_local.transpose() << std::endl;  
            
            len_base_local = base_local.norm();
            double len_end_pose_local = end_pose_local.norm();
            double len_diff =  len_end_pose_local - len_base_local;
            len_diff_delta = len_diff / ((double)mConfig.mNumPosesPerPrim-1);
            len_scale_factor = (len_end_pose_local / len_base_local - 1) / ((double)mConfig.mNumPosesPerPrim-1);
            
            ss << "len base: " << len_base_local << ", len end pose local: " << 
                    len_end_pose_local << ", len_delta: " << len_diff_delta << 
                    ", len_scale_factor " << len_scale_factor << std::endl;
            
            // Calculate real (may changed because of discretization) angle between both vectors.
            double angle = acos(base_local.dot(end_pose_local) / (len_base_local * len_end_pose_local));
            // Add the direction of rotation.
            angle = discrete_rot_diff < 0 ? -angle : angle; 
            angle_delta = angle / ((double)mConfig.mNumPosesPerPrim-1);
            
            ss << "Angle between vectors: " << angle << ", angle delta " << angle_delta << std::endl;
        }
        
        for(unsigned int i=0; i<mConfig.mNumPosesPerPrim; i++) { 
            switch(it->mMovType) {
                // Forward, backward or lateral movement, orientation does not change.
                case MOV_FORWARD:
                case MOV_BACKWARD:
                case MOV_LATERAL: {
                    intermediate_pose[0] = i * x_step;
                    intermediate_pose[1] = i * y_step;
                    intermediate_pose[2] = start_orientation_local;
                    break;
                }
                case MOV_POINTTURN: {
                    intermediate_pose[0] = end_pose_local[0];
                    intermediate_pose[1] = end_pose_local[1];
                    intermediate_pose[2] = start_orientation_local + i * theta_step;
                    break;
                }
                case MOV_FORWARD_TURN:
                case MOV_BACKWARD_TURN: {
                    // Calculate each intermediate pose within the center of rotation frame.
                    base::samples::RigidBodyState rbs_intermediate;
                    base::Quaterniond cur_rot;
                    cur_rot = Eigen::AngleAxis<double>(i * angle_delta, Eigen::Vector3d::UnitZ());
                    //rbs_intermediate.position = cur_rot * base::Vector3d(0,-(len_base_local + i * len_diff_delta), 0);
                    rbs_intermediate.position = cur_rot *  (base_local * (1.0 + len_scale_factor * i));
                    rbs_intermediate.orientation = cur_rot;
                    
                    // Transform back into the base frame.
                    rbs_intermediate.setTransform(cor2base * rbs_intermediate.getTransform() );
                    intermediate_pose[0] = rbs_intermediate.position[0];
                    intermediate_pose[1] = rbs_intermediate.position[1];
                    intermediate_pose[2] = rbs_intermediate.getYaw();
                    break;
                }
                default: {
                    LOG_WARN("Unknown movement type %d during intermediate pose calculation", (int)it->mMovType);
                    break;
                }
            }
            
            // Truncate orientation of intermediate poses to (-PI,PI] (according to OMPL).
            while(intermediate_pose[2] <= -M_PI)
                intermediate_pose[2] += 2*M_PI;
            while(intermediate_pose[2] > M_PI)
                intermediate_pose[2] -= 2*M_PI;
            
            ss << "Intermediate pose (x,y,theta) has been added: " << 
                    intermediate_pose[0] << ", " << 
                    intermediate_pose[1] << ", " << 
                    intermediate_pose[2] << std::endl;
            
            it->mIntermediatePoses.push_back(intermediate_pose);
        }
        //it->mIntermediatePoses.push_back(end_pose_local);
    }
    //printf("%s", ss.str().c_str());
}

void SbplMotionPrimitives::storeToFile(std::string path) {
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

/**
    * Forward and turning speed does already contain the scale factor.
    * Uses grid_local.
    */
bool SbplMotionPrimitives::createCurvePrimForAngle0(double const forward_speed, double const turning_speed, 
        int const prim_id, int const multiplier, Primitive& primitive) {
    
    // TODO Is this radius calculation correct?
    double turning_radius = forward_speed / turning_speed;
    
    // Avoid using too sharp curves.
    if(mConfig.mSpeeds.mMinTurningRadius > 0 && 
            fabs(turning_radius) < mConfig.mSpeeds.mMinTurningRadius) {
        LOG_WARN("Turning radius of %4.2f (forward speed %4.2f, turning speed %4.2f) is too sharp for the system and will be ignored.",
               turning_radius, forward_speed, turning_speed); 
        return false;
    }
    
    base::Vector3d center_of_rotation(0.0, turning_radius, 0.0);
    base::Vector3d vec_endpos(0.0, 0.0, 0.0);
    vec_endpos -= center_of_rotation;
    vec_endpos = Eigen::AngleAxis<double>(turning_speed,          
            Eigen::Vector3d::UnitZ()) * vec_endpos;
    vec_endpos += center_of_rotation;
    // Adds the end orientation.
    vec_endpos[2] = turning_speed;
    
    enum MovementType mov_type = forward_speed > 0 ? MOV_FORWARD_TURN :  MOV_BACKWARD_TURN;
    
    primitive = Primitive(prim_id, 
        0,
        vec_endpos,  
        multiplier, 
        mov_type);
    // Store center of rotation.
    primitive.mCenterOfRotationLocal = center_of_rotation;
    
    return true;
}

bool SbplMotionPrimitives::getSpeeds(unsigned int prim_id, struct Speeds& speeds) {
    if(prim_id > mMapPrimID2Speeds.size()-1) {
        LOG_WARN("Prim id %d unknown, no speed structure available");
        return false;
    }
    speeds = mMapPrimID2Speeds[prim_id];
    return true;
}

int SbplMotionPrimitives::calcDiscreteEndOrientation(double yaw_rad) {
    int discrete_theta = round(yaw_rad / (double)mConfig.mNumAngles);
    while (discrete_theta >= (int)mConfig.mNumAngles)
        discrete_theta -= mConfig.mNumAngles;
    while (discrete_theta < 0)
        discrete_theta += mConfig.mNumAngles;
    return discrete_theta;
}

} // end namespace motion_planning_libraries
