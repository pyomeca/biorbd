
#ifndef BIORBD_C_BINDER
#define BIORBD_C_BINDER

#include "BiorbdModel.h"
#include "biorbdConfig.h"

namespace biorbd {
namespace rigidbody {
#ifndef SKIP_KALMAN
class KalmanReconsIMU;
#endif
}

}
extern "C" { 
	// Create a pointer on a model
    BIORBD_API biorbd::Model* c_biorbdModel(
            const char* pathToModel);
    BIORBD_API void c_deleteBiorbdModel(
            biorbd::Model*);
    BIORBD_API void c_writeBiorbdModel(
            biorbd::Model*, const char * path);


    // Joints functions
    BIORBD_API void c_boneRotationSequence( // Return the angle sequence of a bone named segName
            biorbd::Model* m,
            const char* segName,
            char* seq);
    BIORBD_API void c_localJCS( // Return the LCS for segment of index i in parent coordinate system
            biorbd::Model* m,
            int i,
            double* RtOut);
    BIORBD_API void c_globalJCS(
            biorbd::Model*,
            const double* Q,
            double* jcs);


    // dof functions
    BIORBD_API int c_nQ(
            biorbd::Model* model);
    BIORBD_API int c_nQDot(
            biorbd::Model* model);
    BIORBD_API int c_nQDDot(
            biorbd::Model* model);
    BIORBD_API int c_nGeneralizedTorque(
            biorbd::Model* model);


    // Markers functions
    BIORBD_API int c_nMarkers(
            biorbd::Model* model);
    BIORBD_API void c_markersInLocal(
            biorbd::Model* model,
            double* markPos);
    BIORBD_API void c_markers(
            biorbd::Model* model,
            const double* Q,
            double* markPos,
            bool removeAxis = true,
            bool updateKin = true);
    BIORBD_API void c_addMarker(
            biorbd::Model *model,
            const double *markPos,
            const char* name = "",
            const char* parentName = "",
            bool technical = true,
            bool anatomical = true,
            const char* axesToRemove = "");

    // IMUs functions
    BIORBD_API int c_nIMUs(
            biorbd::Model*);
    BIORBD_API void c_addIMU(
            biorbd::Model *model,
            const double *imuRT,
            const char* name = "",
            const char* parentName = "",
            bool technical = true,
            bool anatomical = true);

    // Kalman IMU
#ifndef SKIP_KALMAN
    BIORBD_API biorbd::rigidbody::KalmanReconsIMU* c_BiorbdKalmanReconsIMU(
            biorbd::Model*,
            double* QinitialGuess = nullptr,
            double freq = 100,
            double noiseF = 5e-3,
            double errorF = 1e-10);
    BIORBD_API void c_deleteBiorbdKalmanReconsIMU(
            biorbd::rigidbody::KalmanReconsIMU*);
    BIORBD_API void c_BiorbdKalmanReconsIMUstep(
            biorbd::Model*,
            biorbd::rigidbody::KalmanReconsIMU*,
            double* imu,
            double* Q = nullptr,
            double* QDot = nullptr,
            double* QDDot = nullptr);
#endif

    // Math functions
    BIORBD_API void c_matrixMultiplication(
            const double* M1,
            const double* M2,
            double* Mout);
    BIORBD_API void c_meanRT(
            const double *imuRT,
            unsigned int nFrame,
            double* imuRT_mean);
    BIORBD_API void c_projectJCSinParentBaseCoordinate(
            const double* parent,
            const double* jcs,
            double * out);
    BIORBD_API void c_transformMatrixToCardan(
            const double* M,
            const char* sequence,
            double* cardanOut);
}

// Fonctions de dispatch pour les données d'entré et de sortie
biorbd::utils::Node3d dispatchMarkersInput(
        const double * pos);
void dispatchMarkersOutput(
        const std::vector<biorbd::rigidbody::NodeBone> &allMarkers,
        double* markers);
biorbd::rigidbody::GeneralizedCoordinates dispatchQinput(
        biorbd::Model* model,
        const double* Q);
void dispatchQoutput(
        const biorbd::rigidbody::GeneralizedCoordinates &eQ,
        double* Q);
void dispatchDoubleOutput(
        const biorbd::utils::Vector&,
        double*);
biorbd::utils::RotoTrans dispatchRTinput(
        const double* rt);
void dispatchRToutput(
        const biorbd::utils::RotoTrans& rt_in,
        double* rt_out);
void dispatchRToutput(
        const std::vector<biorbd::utils::RotoTrans>& rt_in,
        double* rt_out);


//#ifdef UNITY
//// Spécifique à des projets (IMU sous Unity)
//#include "IMU_Unity_Optim.h"
//extern "C" {
//    BIORBD_API void c_alignSpecificAxisWithParentVertical(
//            const double* parentRT,
//            const double * childRT,
//            int idxAxe,
//            double * rotation);
//}
//#endif


#endif // BIORBD_C_BINDER
