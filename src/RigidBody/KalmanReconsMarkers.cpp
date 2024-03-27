#define BIORBD_API_EXPORTS
#include "RigidBody/KalmanReconsMarkers.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "BiorbdModel.h"
#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/NodeSegment.h"

#include <math.h>

using namespace BIORBD_NAMESPACE;

rigidbody::KalmanReconsMarkers::KalmanReconsMarkers() :
    rigidbody::KalmanRecons(),
    m_PpInitial(std::make_shared<utils::Matrix>()),
    m_firstIteration(std::make_shared<bool>(true))
{

}

rigidbody::KalmanReconsMarkers::KalmanReconsMarkers(
    Model &model,
    rigidbody::KalmanParam params) :
    rigidbody::KalmanRecons(model, model.nbTechnicalMarkers()*3, params),
    m_PpInitial(std::make_shared<utils::Matrix>()),
    m_firstIteration(std::make_shared<bool>(true))
{

    // Initialize the filter
    initialize();

}

rigidbody::KalmanReconsMarkers
rigidbody::KalmanReconsMarkers::DeepCopy() const
{
    rigidbody::KalmanReconsMarkers copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::KalmanReconsMarkers::DeepCopy(const
        rigidbody::KalmanReconsMarkers &other)
{
    rigidbody::KalmanRecons::DeepCopy(other);
    *m_PpInitial = *other.m_PpInitial;
    *m_firstIteration = *other.m_firstIteration;
}

void rigidbody::KalmanReconsMarkers::initialize()
{
    rigidbody::KalmanRecons::initialize();

    // Keep in mind the initial m_Pp
    *m_PpInitial = *m_Pp;
}

void rigidbody::KalmanReconsMarkers::manageOcclusionDuringIteration(
    utils::Matrix &InvTp,
    utils::Vector &measure,
    const std::vector<size_t> &occlusion)
{
    for (size_t i = 0; i < occlusion.size(); ++i)
        for (size_t j=occlusion[i] * 3; j< occlusion[i] * 3+3; ++j) {
            InvTp(j,j) =
                0; // Artefact due to the fact that m_R has a value at (j:j+2,j:j+2)
            measure[j] = 0;
        }
}

bool rigidbody::KalmanReconsMarkers::first()
{
    return *m_firstIteration;
}

void rigidbody::KalmanReconsMarkers::reconstructFrame(
    Model &model,
    const rigidbody::Markers &Tobs,
    rigidbody::GeneralizedCoordinates *Q,
    rigidbody::GeneralizedVelocity *Qdot,
    rigidbody::GeneralizedAcceleration *Qddot,
    bool removeAxes)
{
    // Separate the tobs in a big vector
    utils::Vector T(3*Tobs.nbMarkers());
    for (size_t i=0; i<Tobs.nbMarkers(); ++i) {
        T.block(i*3, 0, 3, 1) = Tobs.marker(i);
    }

    // Reconstruct the kinematics
    reconstructFrame(model, T, Q, Qdot, Qddot, removeAxes);
}

void rigidbody::KalmanReconsMarkers::reconstructFrame(
    Model &model,
    const std::vector<rigidbody::NodeSegment> &Tobs,
    rigidbody::GeneralizedCoordinates *Q,
    rigidbody::GeneralizedVelocity *Qdot,
    rigidbody::GeneralizedAcceleration *Qddot,
    bool removeAxes)
{
    // Separate the tobs in a big vector
    utils::Vector T(static_cast<size_t>(3*Tobs.size()));
    for (size_t i=0; i<Tobs.size(); ++i) {
        T.block(i*3, 0, 3, 1) = Tobs[i];
    }

    // Reconstruct the kinematics
    reconstructFrame(model, T, Q, Qdot, Qddot, removeAxes);
}


void rigidbody::KalmanReconsMarkers::reconstructFrame(
    Model &model,
    const utils::Vector &Tobs,
    rigidbody::GeneralizedCoordinates *Q,
    rigidbody::GeneralizedVelocity *Qdot,
    rigidbody::GeneralizedAcceleration *Qddot,
    bool removeAxes)
{
    // An iteration of the Kalman filter
    if (*m_firstIteration) {
        *m_firstIteration = false;
        utils::Vector TobsTP(Tobs);
        TobsTP.block(3*model.nbTechnicalMarkers(0), 0,
                     3*model.nbTechnicalMarkers()-3*model.nbTechnicalMarkers(0), 1) =
                         utils::Vector::Zero(3*model.nbTechnicalMarkers()
                                 -3*model.nbTechnicalMarkers(
                                     0)); // Only keep the markers of the root
        for (size_t j = 0; j < 2; ++j) { // Do the root and then the rest of the body
            if (j != 0) {
                TobsTP = Tobs;    // Re-take all the markers
            }

            for (size_t i=0; i<50; ++i) {
                // The first time, call in a recursive manner to get a descent initial position
                reconstructFrame(model, TobsTP, nullptr, nullptr, nullptr);

                // Reset Pp to initial (we are not interested in the velocity to get to the initial position)
                *m_Pp = *m_PpInitial;
                m_xp->block(*m_nbDof, 0, *m_nbDof*2, 1) = 
                    utils::Vector::Zero(*m_nbDof*2); // Set velocity and acceleration to zero
            }
        }
    }

    // Projected state
    const utils::Vector& xkm(*m_A * *m_xp);
    const rigidbody::GeneralizedCoordinates& Q_tp(xkm.topRows(*m_nbDof));
    Model& updatedModel = static_cast<Model&>(model.UpdateKinematicsCustom (&Q_tp));

    // Projected markers
    const std::vector<rigidbody::NodeSegment>& zest_tp(
        updatedModel.technicalMarkers(Q_tp, removeAxes, false));
    // Jacobian
    const  std::vector<utils::Matrix>& J_tp(updatedModel.technicalMarkersJacobian(
                Q_tp, removeAxes, false));
    // Create only one matrix for zest and Jacobian
    utils::Matrix H(utils::Matrix::Zero(*m_nMeasure,
                            *m_nbDof*3)); // 3*nMarkers => X,Y,Z ; 3*nbDof => Q, Qdot, Qddot
    utils::Vector zest(utils::Vector::Zero(*m_nMeasure));
    std::vector<size_t> occlusionIdx;
    for (size_t i=0; i<*m_nMeasure/3;
            ++i) // Divided by 3 because we are integrate once xyz
    
    if (
        // If there is a marker
#ifdef BIORBD_USE_CASADI_MATH
        !Tobs(i*3).is_zero() && !Tobs(i*3+1).is_zero() && !Tobs(i*3+2).is_zero()
#else
            Tobs(i*3)*Tobs(i*3) + Tobs(i*3+1)*Tobs(i*3+1) + Tobs(i*3+2)*Tobs(i*3+2) != 0.0 &&
                !isnan(Tobs(i*3)*Tobs(i*3) + Tobs(i*3+1)*Tobs(i*3+1) + Tobs(i*3+2)*Tobs(i*3+2))
#endif
        ) {
            H.block(i*3,0,3,*m_nbDof) = J_tp[i];
            zest.block(i*3, 0, 3, 1) = zest_tp[i];
        } else {
            occlusionIdx.push_back(i);
        }

    // Filter
    iteration(Tobs, zest, H, occlusionIdx);

    getState(Q, Qdot, Qddot);
}

void rigidbody::KalmanReconsMarkers::reconstructFrame()
{
    utils::Error::raise("Implémentation impossible");
}
