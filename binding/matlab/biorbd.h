#ifndef MATLAB_BIORBD_H
#define MATLAB_BIORBD_H

#include <mex.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iostream>

#include "s2mMusculoSkeletalModel.h"
#include "s2mRead.h"
#include "class_handle.h"
#include "s2mKalmanReconsMarkers.h"
#include "s2mKalmanReconsIMU.h"
#include "s2mMuscleOptimisation.h"
#include "s2mMatrix.h"

#include "S2M_help.h"
#include "S2M_new.h"
#include "S2M_delete.h"
#include "S2M_changeGravity.h"
#include "S2M_totalMass.h"
#include "S2M_massMatrix.h"
#include "S2M_nControl.h"
#include "S2M_nQ.h"
#include "S2M_nQdot.h"
#include "S2M_nQddot.h"
#include "S2M_nameDof.h"
#include "S2M_nRoot.h"
#include "S2M_nTags.h"
#include "S2M_nameTags.h"
#include "S2M_nBody.h"
#include "S2M_nameBody.h"
#include "S2M_segmentMass.h"
#include "S2M_Tags.h"
#include "S2M_ProjectCustomPoint.h"
#include "S2M_ProjectPoint.h"
#include "S2M_ProjectPointJacobian.h"
#include "S2M_nIMU.h"
#include "S2M_nameIMU.h"
#include "S2M_IMU.h"
#include "S2M_IMUJacobian.h"
#include "S2M_LocalTags.h"
#include "S2M_localJCS.h"
#include "S2M_segmentTags.h"
#include "S2M_inverseKinematics.h"
#include "S2M_inverseKinematicsEKF.h"
#include "S2M_inverseKinematicsEKF_IMU.h"
#include "S2M_Mesh.h"
#include "S2M_Patch.h"
#include "S2M_TagsJacobian.h"
#include "S2M_ContactsPosition.h"
#include "S2M_ContactJacobian.h"
#include "S2M_ContactGamma.h"
#include "S2M_CoM.h"
#include "S2M_CoMJacobian.h"
#include "S2M_CoMdot.h"
#include "S2M_CoMddot.h"
#include "S2M_segmentCoM.h"
#include "S2M_segmentCoMdot.h"
#include "S2M_segmentCoMddot.h"
#include "S2M_CoMangularMomentum.h"
#include "S2M_segmentAngularMomentum.h"
#include "S2M_segmentsInertia.h"
#include "S2M_segmentsVelocities.h"
#include "S2M_globalJCS.h"
#include "S2M_muscleUpdate.h"
#include "S2M_MusclesPoints.h"
#include "S2M_MusclesJacobian.h"
#include "S2M_MusclesLength.h"
#include "S2M_muscleLengthJacobian.h"
#include "S2M_muscleVelocity.h"
#include "S2M_MusclesNames.h"
#include "S2M_MusclesParentNames.h"
#include "S2M_nMuscles.h"
#include "S2M_muscleJointTorqueFromActivation.h"
#include "S2M_muscleJointTorqueFromExcitation.h"
#include "S2M_muscleJointTorqueFromMuscleForce.h"
#include "S2M_MusclesForce.h"
#include "S2M_MusclesForceMax.h"
#include "S2M_muscleForcesNorm.h"
#include "S2M_MusclesActivationDot.h"
#include "S2M_MusclesExcitationDotBuchanan.h"
#include "S2M_ChangeShapeFactors.h"
#include "S2M_MaximeMuscleOptim.h"
#include "S2M_parent.h"

#include "S2M_inverseDynamics.h"
#include "S2M_torqueActivation.h"
#include "S2M_NLeffects.h"
#include "S2M_forwardDynamics.h"
#include "S2M_computeQdot.h"

#ifdef _WIN32
// This is a hack because Eigen can't be dynamically compiled on Windows, while dlib needs consistency in compilation. 
// Please note that this can result in undefined behavior while using s2mMuscleOptimisation...
const int USER_ERROR__inconsistent_build_configuration__see_dlib_faq_1_ = 0;
const int DLIB_VERSION_MISMATCH_CHECK__EXPECTED_VERSION_19_10_0 = 0;
#endif

std::string toLower(const std::string &str);

// MATLAB INTERFACE 
/** \brief Entry point for the muscod application */
void mexFunction( int nlhs, mxArray *plhs[], 
                  int nrhs, const mxArray*prhs[] );

#endif // MATLAB_BIORBD_H
