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
        utils::Error::check(getMuscleGroupId(name)==-1,
                                    "Muscle group already defined");
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

const std::vector<std::shared_ptr<internal_forces::muscles::Muscle>>
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

const internal_forces::muscles::Muscle &internal_forces::muscles::Muscles::muscle(
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
    utils::Error::check(idx<nbMuscleGroups(),
                                "Idx asked is higher than number of muscle groups");
    return (*m_mus)[idx];
}

const internal_forces::muscles::MuscleGroup &internal_forces::muscles::Muscles::muscleGroup(
    size_t idx) const
{
    utils::Error::check(idx<nbMuscleGroups(),
                                "Idx asked is higher than number of muscle groups");
    return (*m_mus)[idx];
}
const internal_forces::muscles::MuscleGroup &internal_forces::muscles::Muscles::muscleGroup(
    const utils::String& name) const
{
    int idx = getMuscleGroupId(name);
    utils::Error::check(idx!=-1, "Group name could not be found");
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
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot)
{

    // Update the muscular position
    updateMuscles(Q, Qdot, true);

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
    const std::vector<std::shared_ptr<internal_forces::muscles::State>>& emg,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot)
{
    return muscularJointTorque(muscleForces(emg, Q, Qdot));
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
    const std::vector<std::shared_ptr<internal_forces::muscles::State>> &emg,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot)
{
    // Update the muscular position
    updateMuscles(Q, Qdot, true);

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
    const rigidbody::GeneralizedCoordinates &Q)
{
    // Update the muscular position
    updateMuscles(Q, true);
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
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
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

    for (auto group : *m_mus) // muscle group
        for (size_t j=0; j<group.nbMuscles(); ++j) {
            group.muscle(j).updateOrientations(model, Q, Qdot, updateKinTP);
#ifndef BIORBD_USE_CASADI_MATH
            if (updateKinTP){
                updateKinTP=1;
            }
#endif
        }
}
void internal_forces::muscles::Muscles::updateMuscles(
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
        for (size_t j=0; j<group.nbMuscles(); ++j) {
            group.muscle(j).updateOrientations(model, Q,updateKinTP);
#ifndef BIORBD_USE_CASADI_MATH
            if (updateKinTP){
                updateKinTP=1;
            }
#endif
        }
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
