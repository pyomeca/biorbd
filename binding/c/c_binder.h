
#ifndef __BIORBD_C_BINDER__
#define __BIORBD_C_BINDER__

#include "s2mMusculoSkeletalModel.h"
#include "biorbd::rigidbody::KalmanReconsIMU.h"
#include <iostream>
#include <fstream>

// Déclare for dllexport
#ifdef _WIN32
 #ifndef S2MLIBRARY_API
  #define S2MLIBRARY_API __declspec(dllexport)
  #define S2MLIBRARY_UNITY_API __declspec(dllexport)
 #endif
#else
 #ifndef S2MLIBRARY_API
  #define S2MLIBRARY_API
  #define S2MLIBRARY_UNITY_API
 #endif
#endif 

extern "C" { 
	// Create a pointer on a model
	S2MLIBRARY_API s2mMusculoSkeletalModel* c_s2mMusculoSkeletalModel(const char* pathToModel);
	S2MLIBRARY_API void c_deleteS2mMusculoSkeletalModel(s2mMusculoSkeletalModel*);  
	S2MLIBRARY_API void c_writeS2mMusculoSkeletalModel(s2mMusculoSkeletalModel*, const char * path);  
	
    // IMUs functions
    S2MLIBRARY_API int c_nIMUs(s2mMusculoSkeletalModel*);
    S2MLIBRARY_API void c_addIMU(s2mMusculoSkeletalModel *model, const double *imuRT, const char* name = "", const char* parentName = "", bool technical = true, bool anatomical = true);
    S2MLIBRARY_API void c_meanIMU(const double *imuRT, unsigned int nFrame, double* imuRT_mean);
    S2MLIBRARY_API biorbd::rigidbody::KalmanReconsIMU* c_biorbd::rigidbody::KalmanReconsIMU(s2mMusculoSkeletalModel*, double* QinitialGuess = NULL, double freq = 100, double noiseF = 5e-3, double errorF = 1e-10);
	S2MLIBRARY_API void c_deletebiorbd::rigidbody::KalmanReconsIMU(biorbd::rigidbody::KalmanReconsIMU*);
    S2MLIBRARY_API void c_biorbd::rigidbody::KalmanReconsIMUstep(s2mMusculoSkeletalModel*, biorbd::rigidbody::KalmanReconsIMU*, double* imu, double* Q = NULL, double* QDot = NULL, double* QDDot = NULL);
	
    // Joints functions
    S2MLIBRARY_API void c_globalJCS(s2mMusculoSkeletalModel*, const double* Q, double* jcs);
    S2MLIBRARY_API void c_projectJCSinParentBaseCoordinate(const double* parent, const double* jcs, double * out);
    S2MLIBRARY_API void c_matrixMultiplication(const double* M1, const double* M2, double* Mout);
    S2MLIBRARY_API void c_localJCS(s2mMusculoSkeletalModel* m, int i, double* RtOut);  // Return the LCS for segment of index i in parent coordinate system
    S2MLIBRARY_API void c_boneRotationSequence(s2mMusculoSkeletalModel* m, const char* segName, char* seq);  // Return the angle sequence of a bone named segName

	// dof functions
	S2MLIBRARY_API int c_nQ(s2mMusculoSkeletalModel* model);  
	S2MLIBRARY_API int c_nQDot(s2mMusculoSkeletalModel* model); 
	S2MLIBRARY_API int c_nQDDot(s2mMusculoSkeletalModel* model); 
	S2MLIBRARY_API int c_nQTau(s2mMusculoSkeletalModel* model); 

	// Markers functions
	S2MLIBRARY_API int c_nTags(s2mMusculoSkeletalModel* model);  
	S2MLIBRARY_API void c_Tags(s2mMusculoSkeletalModel* model, const double* Q, double* markPos, bool removeAxis = true, bool updateKin = true); 
    S2MLIBRARY_API void c_TagsInLocal(s2mMusculoSkeletalModel* model, double* markPos); 
    S2MLIBRARY_API void c_addTags(s2mMusculoSkeletalModel *model, const double *markPos, const char* name = "", const char* parentName = "", bool technical = true, bool anatomical = true, const char* axesToRemove = "");

    // Maths functions
    S2MLIBRARY_API void c_transformMatrixToCardan(const double* M, const char* sequence, double* cardanOut);
}

// Fonctions de dispatch pour les données d'entré et de sortie
Eigen::Vector3d dispatchTagsInput(const double * pos);
void dispatchTagsOutput(const std::vector<biorbd::rigidbody::NodeBone> &allTags, double* tags);
GenCoord dispatchQinput(s2mMusculoSkeletalModel* model, const double*Q);
void dispatchQoutput(const GenCoord &eQ, double*Q);
void dispatchDoubleOutput(const Eigen::VectorXd&, double*);
Attitude dispatchRTinput(const double* rt);
void dispatchRToutput(const Attitude& rt_in, double* rt_out);
void dispatchRToutput(const std::vector<Attitude>& rt_in, double* rt_out);



// Spécifique à des projets (IMU sous Unity)
#include "biorbd::rigidbody::IMU_Unity_Optim.h"
extern "C" { 
	S2MLIBRARY_UNITY_API void c_alignSpecificAxisWithParentVertical(const double* parentRT, const double * childRT, int idxAxe, double * rotation);
}


#endif // __BIORBD_C_BINDER__
