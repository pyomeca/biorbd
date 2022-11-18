#define BIORBD_API_EXPORTS

#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedTorque.h"
#include "InternalForces/Ligaments/Ligament.h"
#include "InternalForces/Ligaments/Ligaments.h"

using namespace BIORBD_NAMESPACE;

internal_forces::ligaments::Ligaments::Ligaments() :
    m_ligaments(std::make_shared<std::vector<internal_forces::ligaments::Ligament>>())
{

}

internal_forces::ligaments::Ligaments::Ligaments(const internal_forces::ligaments::Ligaments &other) :
    m_ligaments(other.m_ligaments)
{

}

internal_forces::ligaments::Ligaments::~Ligaments()
{

}


internal_forces::ligaments::Ligaments internal_forces::ligaments::Ligaments::DeepCopy() const
{
    internal_forces::ligaments::Ligaments copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::ligaments::Ligaments::DeepCopy(const internal_forces::ligaments::Ligaments &other)
{
    m_ligaments->resize(other.m_ligaments->size());
    for (unsigned int i=0; i<other.m_ligaments->size(); ++i) {
        (*m_ligaments)[i] = (*other.m_ligaments)[i];
    }
}

const internal_forces::ligaments::Ligament &internal_forces::ligaments::Ligaments::ligament(
    unsigned int idx) const
{
    utils::Error::check(idx<nbLigaments(),
                                "Idx asked is higher than number of ligaments");
    return (*m_ligaments)[idx];
}

std::vector<utils::String> internal_forces::ligaments::Ligaments::ligamentNames() const
{
    std::vector<utils::String> names;
    for (unsigned int i=0; i<m_ligaments->size(); ++i)
    {
            names.push_back(*m_ligaments[i].name());
    }
    return names;
}

// From muscle activation (return muscle force)
rigidbody::GeneralizedTorque
internal_forces::ligaments::Ligaments::ligamentJointTorque(
    const utils::Vector &F)
{
    // Get the Jacobian matrix and get the forces of each muscle
    const utils::Matrix& jaco(ligamentLengthJacobian());

    // Compute the reaction of the forces on the bodies
    return rigidbody::GeneralizedTorque( -jaco.transpose() * F );
}

// From Muscular Force
rigidbody::GeneralizedTorque
internal_forces::ligaments::Ligaments::ligamentJointTorque(
    const utils::Vector &F,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot)
{

    // Update the muscular position
    updateLigaments(Q, QDot, true);

    return ligamentJointTorque(F);
}

utils::Vector internal_forces::ligaments::Ligaments::ligamentForces(
    const std::vector<std::shared_ptr<internal_forces::muscles::State>>& emg)
{
    // Output variable
    utils::Vector forces(nbLigamentTotal());

    unsigned int cmpLig(0);
    for (unsigned int i=0; i<m_ligaments->size(); ++i) { // muscle group
        for (unsigned int j=0; j<(*m_ligaments)[i].nbLigaments(); ++j) {
            forces(cmpLig, 0) = ((*m_ligaments)[i].ligament(j).force(cmpLig));
            ++cmpLig;
        }
    }

    // The forces
    return forces;
}

utils::Matrix internal_forces::ligaments::Ligaments::ligamentLengthJacobian()
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    const rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    utils::Matrix tp(nbLigamentTotal(), model.nbDof());
    unsigned int cmpLig(0);
    for (unsigned int j=0; j<((*m_ligaments)[i]).nbLigaments(); ++j) {
        tp.block(cmpLig++,0,1,model.nbDof()) = ((*m_ligaments)[i]).ligament(
                j).position().jacobianLength();
    }

    return tp;

}

utils::Matrix internal_forces::ligaments::Ligaments::ligamentsLengthJacobian(
    const rigidbody::GeneralizedCoordinates &Q)
{
    // Update the muscular position
    updateLigaments(Q, true);
    return ligamentsLengthJacobian();
}

unsigned int internal_forces::ligaments::Ligaments::nbLigaments() const
{
    return static_cast<unsigned int>(m_ligaments->size());
}

void internal_forces::ligaments::Ligaments::updateLigaments(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& QDot,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    // Update all the muscles
#ifdef BIORBD_USE_CASADI_MATH
    int updateKinTP = 2;
#else
    int updateKinTP;
    if (updateKin) {
        updateKinTP = 2;
    } else {
        updateKinTP = 0;
    }
#endif

    for (unsigned int j=0; j<group.nbLigaments(); ++j) {
            muscle(j).updateOrientations(model, Q, QDot, updateKinTP);
#ifndef BIORBD_USE_CASADI_MATH
            if (updateKinTP){
                updateKinTP=1;
            }
#endif
        }
}
void internal_forces::ligaments::Ligaments::updateLigaments(
    const rigidbody::GeneralizedCoordinates& Q,
    bool updateKin)
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>
                                       (*this);

    // Update all the muscles
#ifdef BIORBD_USE_CASADI_MATH
    int updateKinTP = 2;
#else
    int updateKinTP;
    if (updateKin) {
        updateKinTP = 2;
    } else {
        updateKinTP = 0;
    }
#endif

    // Update all the muscles
    for (auto group : *m_mus) // muscle group
        for (unsigned int j=0; j<group.nbLigaments(); ++j) {
            group.muscle(j).updateOrientations(model, Q,updateKinTP);
#ifndef BIORBD_USE_CASADI_MATH
            if (updateKinTP){
                updateKinTP=1;
            }
#endif
        }
}
void internal_forces::ligaments::Ligaments::updateLigaments(
    std::vector<std::vector<utils::Vector3d>>& musclePointsInGlobal,
    std::vector<utils::Matrix> &jacoPointsInGlobal,
    const rigidbody::GeneralizedVelocity& QDot)
{
    unsigned int cmpMuscle = 0;
    for (auto group : *m_mus) // muscle  group
        for (unsigned int j=0; j<group.nbLigaments(); ++j) {
            group.muscle(j).updateOrientations(musclePointsInGlobal[cmpMuscle],
                                               jacoPointsInGlobal[cmpMuscle], QDot);
            ++cmpMuscle;
        }
}

void internal_forces::ligaments::Ligaments::updateLigaments(
    std::vector<std::vector<utils::Vector3d>>& musclePointsInGlobal,
    std::vector<utils::Matrix> &jacoPointsInGlobal)
{
    // Updater all the muscles
    unsigned int cmpMuscle = 0;
    for (auto group : *m_mus) // muscle group
        for (unsigned int j=0; j<group.nbLigaments(); ++j) {
            group.muscle(j).updateOrientations(musclePointsInGlobal[cmpMuscle],
                                               jacoPointsInGlobal[cmpMuscle]);
            ++cmpMuscle;
        }
}
