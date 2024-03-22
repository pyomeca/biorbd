#define BIORBD_API_EXPORTS
#include "RigidBody/KalmanReconsIMU.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "BiorbdModel.h"
#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "Utils/Rotation.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/IMU.h"

using namespace BIORBD_NAMESPACE;

rigidbody::KalmanReconsIMU::KalmanReconsIMU() :
    rigidbody::KalmanRecons(),
    m_firstIteration(std::make_shared<bool>(true))
{

}

rigidbody::KalmanReconsIMU::KalmanReconsIMU(
    Model &model,
    rigidbody::KalmanParam params) :
    rigidbody::KalmanRecons(model, model.nbTechIMUs()*9, params),
    m_firstIteration(std::make_shared<bool>(true))
{
    // Initialize the filter
    initialize();
}

rigidbody::KalmanReconsIMU
rigidbody::KalmanReconsIMU::DeepCopy() const
{
    rigidbody::KalmanReconsIMU copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::KalmanReconsIMU::DeepCopy(const
        rigidbody::KalmanReconsIMU &other)
{
    rigidbody::KalmanRecons::DeepCopy(other);
    *m_firstIteration = *other.m_firstIteration;
}

void rigidbody::KalmanReconsIMU::manageOcclusionDuringIteration(
    utils::Matrix &InvTp,
    utils::Vector &measure,
    const std::vector<size_t> &occlusion)
{
    for (size_t i = 0; i < occlusion.size(); ++i)
        for (size_t j=occlusion[i] * 9; j< occlusion[i] * 9+9; ++j) {
            InvTp(j,j) =
                0; // Artefact due to the fact that m_R has a value at (j:j+2,j:j+2)
            measure[j] = 0;
        }
}

bool rigidbody::KalmanReconsIMU::first()
{
    return *m_firstIteration;
}

void rigidbody::KalmanReconsIMU::reconstructFrame(
    Model &m,
    const std::vector<rigidbody::IMU> &IMUobs,
    rigidbody::GeneralizedCoordinates *Q,
    rigidbody::GeneralizedVelocity *Qdot,
    rigidbody::GeneralizedAcceleration *Qddot)
{
    // Separate the IMUobs in a big vector
    utils::Vector T(static_cast<size_t> (3*3*IMUobs.size())); // Matrix 3*3 * nbIMU
    for (size_t i=0; i<IMUobs.size(); ++i)
        for (size_t j=0; j<3; ++j) {
            T.block(9*i+3*j, 0, 3, 1) = IMUobs[i].block(0,j,3,1);
        }

    // Reconstruct the kinematics
    reconstructFrame(m, T, Q, Qdot, Qddot);
}


void rigidbody::KalmanReconsIMU::reconstructFrame(
    Model &model,
    const utils::Vector &IMUobs,
    rigidbody::GeneralizedCoordinates *Q,
    rigidbody::GeneralizedVelocity *Qdot,
    rigidbody::GeneralizedAcceleration *Qddot)
{
    // An iteration of the Kalman filter
    if (*m_firstIteration) {
        *m_firstIteration = false;
        for (size_t i=0; i<300; ++i) {
            // The first time, call in a recursive manner to have a decent initial position
            reconstructFrame(model, IMUobs, nullptr, nullptr, nullptr);

            // We can't use the same trick for reinitiliazing speed in pP as IMU are based on velocity
            m_xp->block(*m_nbDof, 0, *m_nbDof*2, 1) = utils::Vector::Zero(*m_nbDof*2);
        }
    }

    // Projected state
    const utils::Vector& xkm(*m_A * *m_xp);
    rigidbody::GeneralizedCoordinates Q_tp(xkm.topRows(*m_nbDof));
    Model& updatedModel = static_cast<Model&>(model.UpdateKinematicsCustom (&Q_tp));

    // Projected markers
    const std::vector<rigidbody::IMU>& zest_tp = updatedModel.technicalIMU(Q_tp, false);
    // Jacobian
    std::vector<utils::Matrix> J_tp = updatedModel.TechnicalIMUJacobian(Q_tp, false);

    // Create only one matrix for zest and Jacobian
    utils::Matrix H(utils::Matrix::Zero(*m_nMeasure, *m_nbDof*3)); // 3*nCentrales => X,Y,Z ; 3*nbDof => Q, Qdot, Qddot
    utils::Vector zest(utils::Vector::Zero(*m_nMeasure));
    std::vector<size_t> occlusionIdx;
    for (size_t i=0; i<*m_nMeasure/9; ++i) {
        utils::Scalar sum = 0;
        for (size_t j = 0; j < 9; ++j) { // Calculate the norm for the 9 components
            sum += IMUobs(i*9+j)*IMUobs(i*9+j);
        }

        if (
#ifdef BIORBD_USE_CASADI_MATH
            true
#else
            sum != 0.0 && !std::isnan(sum)
#endif
        ){ // If there is an IMU (no zero or NaN)
            H.block(i*9,0,9,*m_nbDof) = J_tp[i];
            const utils::Rotation& rot = zest_tp[i].rot();
            for (size_t j = 0; j < 3; ++j) {
                zest.block(i*9+j*3, 0, 3, 1) = rot.block(0, j, 3, 1);
            }
        } else {
            occlusionIdx.push_back(i);
        }
    }

    // Make the filter
    iteration(IMUobs, zest, H, occlusionIdx);

    getState(Q, Qdot, Qddot);
}

void rigidbody::KalmanReconsIMU::reconstructFrame()
{
    utils::Error::raise("Reconstructing kinematics for IMU needs measurements");
}
