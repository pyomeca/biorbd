#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/Muscles.h"

#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedTorque.h"
#include "InternalForces/Muscles/Muscle.h"
#include "InternalForces/Muscles/MuscleGroup.h"
#include "InternalForces/Muscles/StateDynamics.h"

using namespace BIORBD_NAMESPACE;

internal_forces::muscles::Muscles::Muscles() :
    m_mus(std::make_shared<std::vector<internal_forces::muscles::MuscleGroup>>())
{

}

internal_forces::muscles::Muscles::Muscles(const internal_forces::muscles::Muscles &other) :
    m_mus(other.m_mus)
{

}

internal_forces::muscles::Muscles::~Muscles()
{

}


internal_forces::muscles::Muscles internal_forces::muscles::Muscles::DeepCopy() const
{
    internal_forces::muscles::Muscles copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::muscles::Muscles::DeepCopy(const internal_forces::muscles::Muscles &other)
{
    m_mus->resize(other.m_mus->size());
    for (size_t i=0; i<other.m_mus->size(); ++i) {
        (*m_mus)[i] = (*other.m_mus)[i];
    }
}


void internal_forces::muscles::Muscles::addMuscleGroup(
    const utils::String &name,
    const utils::String &originName,
    const utils::String &insertionName)
{
    if (m_mus->size() > 0) {
        utils::Error::check(getMuscleGroupId(name) == -1, "Muscle group already defined");
    }

    m_mus->push_back(internal_forces::muscles::MuscleGroup(name, originName, insertionName));
}

int internal_forces::muscles::Muscles::getMuscleGroupId(const utils::String
        &name) const
{
    for (size_t i=0; i<m_mus->size(); ++i)
        if (!name.compare((*m_mus)[i].name())) {
            return static_cast<int>(i);
        }
    return -1;
}

std::vector<std::shared_ptr<internal_forces::muscles::Muscle>>
        internal_forces::muscles::Muscles::muscles() const
{
    std::vector<std::shared_ptr<internal_forces::muscles::Muscle>> m;
    for (auto group : muscleGroups()) {
        for (auto muscle : group.muscles()) {
            m.push_back(muscle);
        }
    }
    return m;
}

internal_forces::muscles::Muscle &internal_forces::muscles::Muscles::muscle(
    size_t idx) const
{
    for (auto g : muscleGroups()) {
        if (idx >= g.nbMuscles()) {
            idx -= g.nbMuscles();
        } else {
            return g.muscle(idx);
        }
    }
    utils::Error::raise("idx is higher than the number of muscles");
}

std::vector<utils::String> internal_forces::muscles::Muscles::muscleNames() const
{
    std::vector<utils::String> names;
    for (auto group : muscleGroups()) {
        for (auto muscle : group.muscles()) {
            names.push_back(muscle->name());
        }
    }
    return names;
}

std::vector<internal_forces::muscles::MuscleGroup>&
internal_forces::muscles::Muscles::muscleGroups()
{
    return *m_mus;
}

const std::vector<internal_forces::muscles::MuscleGroup>&
internal_forces::muscles::Muscles::muscleGroups() const
{
    return *m_mus;
}

internal_forces::muscles::MuscleGroup &internal_forces::muscles::Muscles::muscleGroup(
    size_t idx)
{
    utils::Error::check(idx < nbMuscleGroups(), "Idx asked is higher than number of muscle groups");
    return (*m_mus)[idx];
}

const internal_forces::muscles::MuscleGroup &internal_forces::muscles::Muscles::muscleGroup(
    size_t idx) const
{
    utils::Error::check(idx < nbMuscleGroups(), "Idx asked is higher than number of muscle groups");
    return (*m_mus)[idx];
}
const internal_forces::muscles::MuscleGroup &internal_forces::muscles::Muscles::muscleGroup(
    const utils::String& name) const
{
    int idx = getMuscleGroupId(name);
    utils::Error::check(idx != -1, "Group name could not be found");
    return muscleGroup(static_cast<size_t>(idx));
}

// From muscle activation (return muscle force)
rigidbody::GeneralizedTorque
internal_forces::muscles::Muscles::muscularJointTorque(
    const utils::Vector &F)
{
    // Get the Jacobian matrix and get the forces of each muscle
    const utils::Matrix& jaco(musclesLengthJacobian());

    // Compute the reaction of the forces on the bodies
    return rigidbody::GeneralizedTorque( -jaco.transpose() * F );
}

// From Muscular Force
rigidbody::GeneralizedTorque
internal_forces::muscles::Muscles::muscularJointTorque(
    const utils::Vector &F,
    rigidbody::Joints &updatedModel,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot, 
    bool updateMuscleParameters)
{
    // Update the muscular position
    if (updateMuscleParameters) updateMuscles(updatedModel, Q, Qdot);
    return muscularJointTorque(F);
}

// From Muscular Force
rigidbody::GeneralizedTorque
internal_forces::muscles::Muscles::muscularJointTorque(
    const utils::Vector &F,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot, 
    int updateKin)
{    
    // Update the muscular position
    if (updateKin >= 1) updateMuscles(Q, Qdot, updateKin >= 2);
    return muscularJointTorque(F);
}

// From muscle activation (return muscle force)
rigidbody::GeneralizedTorque
internal_forces::muscles::Muscles::muscularJointTorque(
    const std::vector<std::shared_ptr<internal_forces::muscles::State>>& emg)
{
    return muscularJointTorque(muscleForces(emg));
}

// From muscle activation (do not return muscle force)
rigidbody::GeneralizedTorque
internal_forces::muscles::Muscles::muscularJointTorque(
    rigidbody::Joints &updatedModel,
    const std::vector<std::shared_ptr<internal_forces::muscles::State>>& emg,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot, 
    bool updateMuscleParameters)
{
    return muscularJointTorque(muscleForces(updatedModel, emg, Q, Qdot, updateMuscleParameters));
}

// From muscle activation (do not return muscle force)
rigidbody::GeneralizedTorque
internal_forces::muscles::Muscles::muscularJointTorque(
    const std::vector<std::shared_ptr<internal_forces::muscles::State>>& emg,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot, 
    int updateKin)
{
    return muscularJointTorque(muscleForces(emg, Q, Qdot, updateKin));
}

utils::Vector internal_forces::muscles::Muscles::activationDot(
    const std::vector<std::shared_ptr<internal_forces::muscles::State>>& emg,
    bool areadyNormalized)
{
    utils::Vector activationDot(nbMuscleTotal());

    size_t cmp(0);
    for (size_t i=0; i<nbMuscleGroups(); ++i)
        for (size_t j=0; j<muscleGroup(i).nbMuscles(); ++j) {
            // Recueillir dérivées d'activtion
            activationDot(static_cast<unsigned int>(cmp)) =
                muscleGroup(i).muscle(j).activationDot(*emg[cmp], areadyNormalized);
            ++cmp;
        }

    return activationDot;
}

utils::Vector internal_forces::muscles::Muscles::muscleForces(
    const std::vector<std::shared_ptr<internal_forces::muscles::State>>& emg)
{
    // Output variable
    utils::Vector forces(nbMuscleTotal());

    size_t cmpMus(0);
    for (size_t i=0; i<m_mus->size(); ++i) { // muscle group
        for (size_t j=0; j<(*m_mus)[i].nbMuscles(); ++j) {
            forces(static_cast<unsigned int>(cmpMus), 0) = ((*m_mus)[i].muscle(j).force(*emg[cmpMus]));
            ++cmpMus;
        }
    }

    // The forces
    return forces;
}

utils::Vector internal_forces::muscles::Muscles::muscleForces(
    rigidbody::Joints &updatedModel,
    const std::vector<std::shared_ptr<internal_forces::muscles::State>> &emg,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot, 
    bool updateMuscleParameters)
{
    // Update the muscular position
    if (updateMuscleParameters) updateMuscles(updatedModel, Q, Qdot);
    return muscleForces(emg);
}

utils::Vector internal_forces::muscles::Muscles::muscleForces(
    const std::vector<std::shared_ptr<internal_forces::muscles::State>> &emg,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot, 
    int updateKin)
{
    // Update the muscular position
    updateMuscles(Q, Qdot, updateKin);
    return muscleForces(emg);
}

size_t internal_forces::muscles::Muscles::nbMuscleGroups() const
{
    return m_mus->size();
}

utils::Matrix internal_forces::muscles::Muscles::musclesLengthJacobian()
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    const rigidbody::Joints &model = dynamic_cast<rigidbody::Joints &>(*this);

    utils::Matrix tp(nbMuscleTotal(), model.nbDof());
    size_t cmpMus(0);
    for (size_t i=0; i<nbMuscleGroups(); ++i)
        for (size_t j=0; j<((*m_mus)[i]).nbMuscles(); ++j) {
            tp.block(static_cast<unsigned int>(cmpMus++),0,1, static_cast<unsigned int>(model.nbDof())) = 
                ((*m_mus)[i]).muscle(j).position().jacobianLength();
        }

    return tp;

}

utils::Matrix internal_forces::muscles::Muscles::musclesLengthJacobian(
    rigidbody::Joints &updatedModel,
    const rigidbody::GeneralizedCoordinates &Q, 
    bool updateMuscleParameters)
{
    // Update the muscular position
    if (updateMuscleParameters) updateMuscles(updatedModel, Q);
    return musclesLengthJacobian();
}

utils::Matrix internal_forces::muscles::Muscles::musclesLengthJacobian(
    const rigidbody::GeneralizedCoordinates &Q, 
    int updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    if (updateKin < 2) {
        utils::Error::raise(
            utils::String("When using Casadi, this method must set updateKin to true. ") +
            "Alternatively, you can call musclesLengthJacobian with the pre-updated model."
        );
    }
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = dynamic_cast<rigidbody::Joints&>(*this).UpdateKinematicsCustom(updateKin >= 2 ? &Q : nullptr);

    // Update the muscular position
    if (updateKin >= 1) updateMuscles(updatedModel, Q);
    return musclesLengthJacobian();
}


size_t internal_forces::muscles::Muscles::nbMuscleTotal() const
{
    return nbMuscles();
}

size_t internal_forces::muscles::Muscles::nbMuscles() const
{
    size_t total(0);
    for (size_t grp=0; grp<m_mus->size(); ++grp) { // muscular group
        total += (*m_mus)[grp].nbMuscles();
    }
    return total;
}

void internal_forces::muscles::Muscles::updateMuscles(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q)
{
    // Update all the muscles
    for (auto group : *m_mus) { // muscle group
        for (size_t j = 0; j < group.nbMuscles(); ++j) {
            group.muscle(j).updateOrientations(updatedModel, Q);
        }
    }
}

void internal_forces::muscles::Muscles::updateMuscles(
    const rigidbody::GeneralizedCoordinates& Q,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    if (!updateKin) {
        utils::Error::raise(
            utils::String("When using Casadi, this method must set updateKin to true. ") +
            "Alternatively, you can call updateMuscles with the pre-updated model."
        );
    }
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = dynamic_cast<rigidbody::Joints&>(*this).UpdateKinematicsCustom(updateKin ? &Q : nullptr);
    updateMuscles(updatedModel, Q);
}

void internal_forces::muscles::Muscles::updateMuscles(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot)
{
    // Update all the muscles
    for (auto group : *m_mus) {// muscle group
        for (size_t j = 0; j < group.nbMuscles(); ++j) {
            group.muscle(j).updateOrientations(updatedModel, Q, Qdot);
        }
    }
}

void internal_forces::muscles::Muscles::updateMuscles(
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    if (!updateKin) {
        utils::Error::raise(
            utils::String("When using Casadi, this method must set updateKin to true. ") +
            "Alternatively, you can call updateMuscles with the pre-updated model."
        );
    }
    rigidbody::Joints
#else
    rigidbody::Joints&
#endif
    updatedModel = dynamic_cast<rigidbody::Joints&>(*this).UpdateKinematicsCustom(updateKin ? &Q : nullptr);

    updateMuscles(updatedModel, Q, Qdot);
}

void internal_forces::muscles::Muscles::updateMuscles(
    std::vector<std::vector<utils::Vector3d>>& musclePointsInGlobal,
    std::vector<utils::Matrix> &jacoPointsInGlobal,
    const rigidbody::GeneralizedVelocity& Qdot)
{
    size_t cmpMuscle = 0;
    for (auto group : *m_mus) // muscle  group
        for (size_t j=0; j<group.nbMuscles(); ++j) {
            group.muscle(j).updateOrientations(
                musclePointsInGlobal[cmpMuscle],
                jacoPointsInGlobal[cmpMuscle], 
                Qdot
            );
            ++cmpMuscle;
        }
}

std::vector<std::shared_ptr<internal_forces::muscles::State>>
        internal_forces::muscles::Muscles::stateSet()
{
    std::vector<std::shared_ptr<internal_forces::muscles::State>> out;
    for (size_t i=0; i<nbMuscles(); ++i) {
        out.push_back(muscle(i).m_state);
    }
    return out;
}

void internal_forces::muscles::Muscles::updateMuscles(
    std::vector<std::vector<utils::Vector3d>>& musclePointsInGlobal,
    std::vector<utils::Matrix> &jacoPointsInGlobal)
{
    // Updater all the muscles
    size_t cmpMuscle = 0;
    for (auto group : *m_mus) // muscle group
        for (size_t j=0; j<group.nbMuscles(); ++j) {
            group.muscle(j).updateOrientations(musclePointsInGlobal[cmpMuscle],
                                               jacoPointsInGlobal[cmpMuscle]);
            ++cmpMuscle;
        }
}
