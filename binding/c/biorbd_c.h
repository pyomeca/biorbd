
#ifndef BIORBD_C_BINDER
#define BIORBD_C_BINDER

#include "biorbdConfig.h"
#include "BiorbdModel.h"
#ifdef _WIN32
    #define BIORBD_API_C __declspec(dllexport)
#else
    #define BIORBD_API_C
#endif

namespace biorbd
{
namespace BIORBD_MATH_NAMESPACE
{
namespace rigidbody
{
#ifdef MODULE_KALMAN
    class KalmanReconsIMU;
#endif
}
}
}

extern "C" {
    // Create a pointer on a model
    BIORBD_API_C biorbd::BIORBD_MATH_NAMESPACE::Model* c_biorbdModel(
        const char* pathToModel);
    BIORBD_API_C void c_deleteBiorbdModel(
        biorbd::BIORBD_MATH_NAMESPACE::Model*);
    BIORBD_API_C void c_writeBiorbdModel(
        biorbd::BIORBD_MATH_NAMESPACE::Model*, const char * path);


    // Joints functions
    BIORBD_API_C void
    c_boneRotationSequence( // Return the angle sequence of a bone named segName
        biorbd::BIORBD_MATH_NAMESPACE::Model* m,
        const char* segName,
        char* seq);
    BIORBD_API_C void
    c_localJCS( // Return the LCS for segment of index i in parent coordinate system
        biorbd::BIORBD_MATH_NAMESPACE::Model* m,
        int i,
        double* RtOut);
    BIORBD_API_C void c_globalJCS(
        biorbd::BIORBD_MATH_NAMESPACE::Model*,
        const double* Q,
        double* jcs);
    BIORBD_API_C void c_inverseDynamics(
        biorbd::BIORBD_MATH_NAMESPACE::Model* model,
        const double* q,
        const double* qdot,
        const double* qddot,
        double* tau);
    BIORBD_API_C void c_massMatrix(
        biorbd::BIORBD_MATH_NAMESPACE::Model* model,
        const double* q,
        double* massMatrix);
    BIORBD_API_C void c_CoM(
        biorbd::BIORBD_MATH_NAMESPACE::Model* model,
        const double* q,
        double *com);


    // dof functions
    BIORBD_API_C int c_nQ(
        biorbd::BIORBD_MATH_NAMESPACE::Model* model);
    BIORBD_API_C int c_nQDot(
        biorbd::BIORBD_MATH_NAMESPACE::Model* model);
    BIORBD_API_C int c_nQDDot(
        biorbd::BIORBD_MATH_NAMESPACE::Model* model);
    BIORBD_API_C int c_nGeneralizedTorque(
        biorbd::BIORBD_MATH_NAMESPACE::Model* model);


    // Markers functions
    BIORBD_API_C int c_nMarkers(
        biorbd::BIORBD_MATH_NAMESPACE::Model* model);
    BIORBD_API_C void c_markersInLocal(
        biorbd::BIORBD_MATH_NAMESPACE::Model* model,
        double* markPos);
    BIORBD_API_C void c_markers(
        biorbd::BIORBD_MATH_NAMESPACE::Model* model,
        const double* Q,
        double* markPos,
        bool removeAxis = true,
        bool updateKin = true);
    BIORBD_API_C void c_addMarker(
        biorbd::BIORBD_MATH_NAMESPACE::Model* model,
        const double *markPos,
        const char* name = "",
        const char* parentName = "",
        bool technical = true,
        bool anatomical = true,
        const char* axesToRemove = "");

    // IMUs functions
    BIORBD_API_C int c_nIMUs(
        biorbd::BIORBD_MATH_NAMESPACE::Model*);
    BIORBD_API_C void c_addIMU(
        biorbd::BIORBD_MATH_NAMESPACE::Model* model,
        const double *imuRT,
        const char* name = "",
        const char* parentName = "",
        bool technical = true,
        bool anatomical = true);

    // Kalman IMU
#ifdef MODULE_KALMAN
    BIORBD_API_C biorbd::BIORBD_MATH_NAMESPACE::rigidbody::KalmanReconsIMU* c_BiorbdKalmanReconsIMU(
        biorbd::BIORBD_MATH_NAMESPACE::Model*,
        double* QinitialGuess = nullptr,
        double freq = 100,
        double noiseF = 5e-3,
        double errorF = 1e-10);
    BIORBD_API_C void c_deleteBiorbdKalmanReconsIMU(
        biorbd::BIORBD_MATH_NAMESPACE::rigidbody::KalmanReconsIMU*);
    BIORBD_API_C void c_BiorbdKalmanReconsIMUstep(
        biorbd::BIORBD_MATH_NAMESPACE::Model*,
        biorbd::BIORBD_MATH_NAMESPACE::rigidbody::KalmanReconsIMU*,
        double* imu,
        double* Q = nullptr,
        double* QDot = nullptr,
        double* QDDot = nullptr);
#endif

    // Math functions
    BIORBD_API_C void c_matrixMultiplication(
        const double* M1,
        const double* M2,
        double* Mout);
    BIORBD_API_C void c_meanRT(
        const double *imuRT,
        unsigned int nFrame,
        double* imuRT_mean);
    BIORBD_API_C void c_projectJCSinParentBaseCoordinate(
        const double* parent,
        const double* jcs,
        double * out);
    BIORBD_API_C void c_transformMatrixToCardan(
        const double* M,
        const char* sequence,
        double* cardanOut);
    BIORBD_API_C void c_solveLinearSystem(
        const double* A,
        int nRows,
        int nCol,
        const double* b,
        double* x);
}

// Fonctions de dispatch pour les données d'entré et de sortie
biorbd::BIORBD_MATH_NAMESPACE::utils::Vector3d dispatchMarkersInput(
    const double * pos);
void dispatchMarkersOutput(
    const std::vector<biorbd::BIORBD_MATH_NAMESPACE::rigidbody::NodeSegment> &allMarkers,
    double* markers);
biorbd::BIORBD_MATH_NAMESPACE::rigidbody::GeneralizedCoordinates dispatchQinput(
    biorbd::BIORBD_MATH_NAMESPACE::Model* model,
    const double* Q);
void dispatchQoutput(
    const biorbd::BIORBD_MATH_NAMESPACE::rigidbody::GeneralizedCoordinates &eQ,
    double* Q);
void dispatchTauOutput(
    const biorbd::BIORBD_MATH_NAMESPACE::rigidbody::GeneralizedTorque &eTau,
    double* Tau);
void dispatchDoubleOutput(
    const biorbd::BIORBD_MATH_NAMESPACE::utils::Vector&,
    double*);
biorbd::BIORBD_MATH_NAMESPACE::utils::RotoTrans dispatchRTinput(
    const double* rt);
void dispatchRToutput(
    const biorbd::BIORBD_MATH_NAMESPACE::utils::RotoTrans& rt_in,
    double* rt_out);
void dispatchRToutput(
    const std::vector<biorbd::BIORBD_MATH_NAMESPACE::utils::RotoTrans>& rt_in,
    double* rt_out);
biorbd::BIORBD_MATH_NAMESPACE::utils::Matrix dispatchMatrixInput(
    const double* matXd,
    int nRows,
    int nCols);
biorbd::BIORBD_MATH_NAMESPACE::utils::Vector dispatchVectorInput(
    const double* vecXd,
    int nElement);
void dispatchVectorOutput(
    const biorbd::BIORBD_MATH_NAMESPACE::utils::Vector& vect,
    double* vect_out);

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
