
#ifndef __BIORBD_C_BINDER__
#define __BIORBD_C_BINDER__

#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "RigidBody/KalmanReconsIMU.h"
#include <iostream>
#include <fstream>

extern "C" { 
	// Create a pointer on a model
    BIORBD_API biorbd::Model* c_biorbd::Model(const char* pathToModel);
    BIORBD_API void c_deletebiorbd::Model(biorbd::Model*);
    BIORBD_API void c_writebiorbd::Model(biorbd::Model*, const char * path);
	
    // IMUs functions
    BIORBD_API int c_nIMUs(biorbd::Model*);
    BIORBD_API void c_addIMU(biorbd::Model *model, const double *imuRT, const char* name = "", const char* parentName = "", bool technical = true, bool anatomical = true);
    BIORBD_API void c_meanIMU(const double *imuRT, unsigned int nFrame, double* imuRT_mean);
    BIORBD_API biorbd::rigidbody::KalmanReconsIMU* c_biorbd::rigidbody::KalmanReconsIMU(biorbd::Model*, double* QinitialGuess = NULL, double freq = 100, double noiseF = 5e-3, double errorF = 1e-10);
    BIORBD_API void c_deletebiorbd::rigidbody::KalmanReconsIMU(biorbd::rigidbody::KalmanReconsIMU*);
    BIORBD_API void c_biorbd::rigidbody::KalmanReconsIMUstep(biorbd::Model*, biorbd::rigidbody::KalmanReconsIMU*, double* imu, double* Q = NULL, double* QDot = NULL, double* QDDot = NULL);
	
    // Joints functions
    BIORBD_API void c_globalJCS(biorbd::Model*, const double* Q, double* jcs);
    BIORBD_API void c_projectJCSinParentBaseCoordinate(const double* parent, const double* jcs, double * out);
    BIORBD_API void c_matrixMultiplication(const double* M1, const double* M2, double* Mout);
    BIORBD_API void c_localJCS(biorbd::Model* m, int i, double* RtOut);  // Return the LCS for segment of index i in parent coordinate system
    BIORBD_API void c_boneRotationSequence(biorbd::Model* m, const char* segName, char* seq);  // Return the angle sequence of a bone named segName

	// dof functions
    BIORBD_API int c_nQ(biorbd::Model* model);
    BIORBD_API int c_nQDot(biorbd::Model* model);
    BIORBD_API int c_nQDDot(biorbd::Model* model);
    BIORBD_API int c_nQTau(biorbd::Model* model);

	// Markers functions
    BIORBD_API int c_nTags(biorbd::Model* model);
    BIORBD_API void c_Tags(biorbd::Model* model, const double* Q, double* markPos, bool removeAxis = true, bool updateKin = true);
    BIORBD_API void c_TagsInLocal(biorbd::Model* model, double* markPos);
    BIORBD_API void c_addTags(biorbd::Model *model, const double *markPos, const char* name = "", const char* parentName = "", bool technical = true, bool anatomical = true, const char* axesToRemove = "");

    // Maths functions
    BIORBD_API void c_transformMatrixToCardan(const double* M, const char* sequence, double* cardanOut);
}

// Fonctions de dispatch pour les données d'entré et de sortie
Eigen::Vector3d dispatchTagsInput(const double * pos);
void dispatchTagsOutput(const std::vector<biorbd::rigidbody::NodeBone> &allTags, double* tags);
GenCoord dispatchQinput(biorbd::Model* model, const double*Q);
void dispatchQoutput(const GenCoord &eQ, double*Q);
void dispatchDoubleOutput(const Eigen::VectorXd&, double*);
Attitude dispatchRTinput(const double* rt);
void dispatchRToutput(const Attitude& rt_in, double* rt_out);
void dispatchRToutput(const std::vector<Attitude>& rt_in, double* rt_out);



// Spécifique à des projets (IMU sous Unity)
#include "biorbd::rigidbody::IMU_Unity_Optim.h"
extern "C" { 
    BIORBD_API void c_alignSpecificAxisWithParentVertical(const double* parentRT, const double * childRT, int idxAxe, double * rotation);
}


#endif // __BIORBD_C_BINDER__
