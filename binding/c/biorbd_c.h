
#ifndef BIORBD_C_BINDER
#define BIORBD_C_BINDER

#include "biorbdConfig.h"

#include "BiorbdModel.h"
#ifdef _WIN32
#define BIORBD_API_C __declspec(dllexport)
#else
#define BIORBD_API_C
#endif

namespace BIORBD_NAMESPACE {
namespace rigidbody {
#ifdef MODULE_KALMAN
class KalmanReconsIMU;
#endif
}  // namespace rigidbody
}  // namespace BIORBD_NAMESPACE

extern "C" {
// Create a pointer on a model
BIORBD_API_C BIORBD_NAMESPACE::Model* c_biorbdModel(const char* pathToModel);
BIORBD_API_C void c_deleteBiorbdModel(BIORBD_NAMESPACE::Model*);
BIORBD_API_C void c_writeBiorbdModel(
    BIORBD_NAMESPACE::Model*,
    const char* path);

// Joints functions
BIORBD_API_C void
    c_boneRotationSequence( // Return the angle sequence of a bone named segName
        BIORBD_NAMESPACE::Model* m,
        const char* segName,
        char* seq);
BIORBD_API_C void
    c_localJCS( // Return the LCS for segment of index i in parent coordinate system
        BIORBD_NAMESPACE::Model* m,
        int i,
        double* RtOut);
BIORBD_API_C void
c_globalJCS(BIORBD_NAMESPACE::Model*, const double* Q, double* jcs);
BIORBD_API_C void c_inverseDynamics(
    BIORBD_NAMESPACE::Model* model,
    const double* q,
    const double* qdot,
    const double* qddot,
    double* tau);
BIORBD_API_C void c_massMatrix(
    BIORBD_NAMESPACE::Model* model,
    const double* q,
    double* massMatrix);
BIORBD_API_C void
c_CoM(BIORBD_NAMESPACE::Model* model, const double* q, double* com);
BIORBD_API_C int c_nSegments(BIORBD_NAMESPACE::Model* model);

// dof functions
BIORBD_API_C int c_nRoot(BIORBD_NAMESPACE::Model* model);
BIORBD_API_C int c_nQ(BIORBD_NAMESPACE::Model* model);
BIORBD_API_C int c_nQDot(BIORBD_NAMESPACE::Model* model);
BIORBD_API_C int c_nQDDot(BIORBD_NAMESPACE::Model* model);
BIORBD_API_C int c_nGeneralizedTorque(BIORBD_NAMESPACE::Model* model);

// Markers functions
BIORBD_API_C int c_nMarkers(BIORBD_NAMESPACE::Model* model);
BIORBD_API_C void c_markersInLocal(
    BIORBD_NAMESPACE::Model* model,
    double* markPos);
BIORBD_API_C void c_markers(
    BIORBD_NAMESPACE::Model* model,
    const double* Q,
    double* markPos,
    bool removeAxis = true,
    bool updateKin = true);
BIORBD_API_C void c_addMarker(
    BIORBD_NAMESPACE::Model* model,
    const double* markPos,
    const char* name = "",
    const char* parentName = "",
    bool technical = true,
    bool anatomical = true,
    const char* axesToRemove = "");

// IMUs functions
BIORBD_API_C int c_nIMUs(BIORBD_NAMESPACE::Model*);
BIORBD_API_C void c_addIMU(
    BIORBD_NAMESPACE::Model* model,
    const double* imuRT,
    const char* name = "",
    const char* parentName = "",
    bool technical = true,
    bool anatomical = true);
BIORBD_API_C void c_IMU(
    BIORBD_NAMESPACE::Model*,
    const double* Q,
    double* output,
    bool updateKin = true);

// Kalman IMU
#ifdef MODULE_KALMAN
BIORBD_API_C BIORBD_NAMESPACE::rigidbody::KalmanReconsIMU*
c_BiorbdKalmanReconsIMU(
    BIORBD_NAMESPACE::Model*,
    double* QinitialGuess = nullptr,
    double freq = 100,
    double noiseF = 5e-3,
    double errorF = 1e-10);
BIORBD_API_C void c_deleteBiorbdKalmanReconsIMU(
    BIORBD_NAMESPACE::rigidbody::KalmanReconsIMU*);
BIORBD_API_C void c_BiorbdKalmanReconsIMUstep(
    BIORBD_NAMESPACE::Model*,
    BIORBD_NAMESPACE::rigidbody::KalmanReconsIMU*,
    double* imu,
    double* Q = nullptr,
    double* QDot = nullptr,
    double* QDDot = nullptr);
#endif

// Math functions
BIORBD_API_C void
c_matrixMultiplication(const double* M1, const double* M2, double* Mout);
BIORBD_API_C void
c_meanRT(const double* imuRT, unsigned int nFrame, double* imuRT_mean);
BIORBD_API_C void c_projectJCSinParentBaseCoordinate(
    const double* parent,
    const double* jcs,
    double* out);
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
BIORBD_NAMESPACE::utils::Vector3d dispatchMarkersInput(const double* pos);
void dispatchMarkersOutput(
    const std::vector<BIORBD_NAMESPACE::rigidbody::NodeSegment>& allMarkers,
    double* markers);
BIORBD_NAMESPACE::rigidbody::GeneralizedCoordinates dispatchQinput(
    BIORBD_NAMESPACE::Model* model,
    const double* Q);
void dispatchQoutput(
    const BIORBD_NAMESPACE::rigidbody::GeneralizedCoordinates& eQ,
    double* Q);
void dispatchTauOutput(
    const BIORBD_NAMESPACE::rigidbody::GeneralizedTorque& eTau,
    double* Tau);
void dispatchDoubleOutput(const BIORBD_NAMESPACE::utils::Vector&, double*);
BIORBD_NAMESPACE::utils::RotoTrans dispatchRTinput(const double* rt);
void dispatchRToutput(
    const BIORBD_NAMESPACE::utils::RotoTrans& rt_in,
    double* rt_out);
void dispatchRToutput(
    const std::vector<BIORBD_NAMESPACE::utils::RotoTrans>& rt_in,
    double* rt_out);
void dispatchRToutput(
    const std::vector<BIORBD_NAMESPACE::rigidbody::IMU>& rt_in,
    double* rt_out);
BIORBD_NAMESPACE::utils::Matrix
dispatchMatrixInput(const double* matXd, int nRows, int nCols);
BIORBD_NAMESPACE::utils::Vector dispatchVectorInput(
    const double* vecXd,
    int nElement);
void dispatchVectorOutput(
    const BIORBD_NAMESPACE::utils::Vector& vect,
    double* vect_out);

// #ifdef UNITY
//// Spécifique à des projets (IMU sous Unity)
// #include "IMU_Unity_Optim.h"
// extern "C" {
//     BIORBD_API void c_alignSpecificAxisWithParentVertical(
//             const double* parentRT,
//             const double * childRT,
//             int idxAxe,
//             double * rotation);
// }
// #endif

#endif  // BIORBD_C_BINDER
