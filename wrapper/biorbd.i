/* File : biorbd.i */
%module biorbd
%{
#include "s2mMusculoSkeletalModel.h"

#include <boost/numeric/odeint.hpp>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>

#include "s2mJoints.h"
#include "s2mJoint.h"
#include "s2mJointMoving.h"
#include "s2mMatrix.h"
#include "s2mNodeWrap.h"
#include "s2mTime.h"
#include "s2mKalmanReconsIMU.h"
#include "s2mKalmanReconsMarkers.h"
#include "s2mMuscleOptimisation.h"
#include "s2mIMU_Unity_Optim.h"
%}

/* Instantiate std_vector */
//%include <std_vector.i>

/* Instantiate std_string */
//%include <std_iostream.i>

// Instantiate templates
namespace std {

}
   

/* Includes all neceressary files from the API */

%include "biorbdConfig.h"
%include "s2mOptions.h"

%include "s2mVector.h"
%include "s2mGenCoord.h"
%include "s2mTau.h"
%include "s2mMatrix.h"
%include "s2mAttitude.h"
%include "s2mNodeAttitude.h"/*
%include "s2mQuaternion.h"
%include "s2mNode.h"
%include "s2mMarkers.h"

%include "s2mPatch.h"
%include "s2mTime.h"
%include "s2mTimer.h"
%include "s2mBenchmark.h"
%include "s2mString.h"
%include "s2mEquation.h"
%include "s2mIfStream.h"
%include "s2mError.h"
%include "s2mPath.h"
%include "s2mRead.h"
%include "s2mIntegrator.h"

%include "s2mJoints.h"
%include "s2mJoint.h"
%include "s2mJointMoving.h"
%include "s2mJointIntraBone.h"

%include "s2mActuator.h"
%include "s2mActuatorConstant.h"
%include "s2mActuatorGauss3p.h"
%include "s2mActuatorGauss6p.h"
%include "s2mActuatorLinear.h"
%include "s2mActuators.h"

%include "s2mNodeBone.h"
%include "s2mBone.h"
%include "s2mBoneCaracteristics.h"
%include "s2mBoneMesh.h"

%include "s2mNodeMuscle.h"
%include "s2mGroupeMusculaire.h"
%include "s2mMuscleCompound.h"
%include "s2mMuscleCaracteristics.h"
%include "s2mMuscle.h"
%include "s2mMuscleForce.h"
%include "s2mMuscleForceFromInsertion.h"
%include "s2mMuscleForceFromOrigin.h"
%include "s2mMuscleGeometry.h"
%include "s2mMuscleHillType.h"
%include "s2mMuscleHillTypeChadwick.h"
%include "s2mMuscleHillTypeMaxime.h"
%include "s2mMuscleHillTypeSchutte.h"
%include "s2mMuscleHillTypeSimple.h"
%include "s2mMuscleHillTypeThelen.h"
%include "s2mMuscleMesh.h"
%include "s2mMuscleMeshTransverse.h"
%include "s2mMusclePathChanger.h"
%include "s2mMusclePathChangers.h"
%include "s2mMuscles.h"
%include "s2mMuscleState.h"
%include "s2mMuscleStateActual.h"
%include "s2mMuscleStateActualBuchanan.h"
%include "s2mMuscleStateMax.h"
%include "s2mMuscleOptimisation.h"

%include "s2mIMU.h"
%include "s2mIMUs.h"

%include "s2mContacts.h"

%include "s2mNodeWrap.h"
%include "s2mViaPoint.h"
%include "s2mWrappingObject.h"
%include "s2mWrappingSphere.h"
%include "s2mWrappingCylinder.h"
%include "s2mWriter.h"

%include "s2mKalmanRecons.h"
%include "s2mKalmanReconsIMU.h"
%include "s2mKalmanReconsMarkers.h"

%include "s2mIMU_Unity_Optim.h"

%include "s2mMusculoSkeletalModel.h"
*/
