#define BIORBD_API_EXPORTS

#include "Utils/Error.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "InternalForces/PathModifiers.h"
#include "InternalForces/Compound.h"
#include "InternalForces/Muscles/MusclesEnums.h"
#include "InternalForces/Muscles/StateDynamics.h"
#include "InternalForces/Muscles/StateDynamicsBuchanan.h"
#include "InternalForces/Muscles/StateDynamicsDeGroote.h"
#include "InternalForces/Muscles/Characteristics.h"
#include "InternalForces/Muscles/MuscleGeometry.h"
#include "InternalForces/Muscles/Muscle.h"

using namespace BIORBD_NAMESPACE;
internal_forces::muscles::Muscle::Muscle() :
    internal_forces::Compound(),
    m_position(std::make_shared<internal_forces::muscles::MuscleGeometry>()),
    m_type(std::make_shared<internal_forces::muscles::MUSCLE_TYPE>(internal_forces::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_characteristics(std::make_shared<internal_forces::muscles::Characteristics>()),
    m_state(std::make_shared<internal_forces::muscles::State>())
{
    setType();

}

internal_forces::muscles::Muscle::Muscle(
    const utils::String & name,
    const internal_forces::muscles::MuscleGeometry & position,
    const internal_forces::muscles::Characteristics &characteristics) :
    internal_forces::Compound (name),
    m_position(std::make_shared<internal_forces::muscles::MuscleGeometry>(position)),
    m_type(std::make_shared<internal_forces::muscles::MUSCLE_TYPE>(internal_forces::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_characteristics(std::make_shared<internal_forces::muscles::Characteristics>
                      (characteristics)),
    m_state(std::make_shared<internal_forces::muscles::State>())
{

}

internal_forces::muscles::Muscle::Muscle(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &position,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::muscles::State &dynamicState) :
    internal_forces::Compound (name),
    m_position(std::make_shared<internal_forces::muscles::MuscleGeometry>(position)),
    m_type(std::make_shared<internal_forces::muscles::MUSCLE_TYPE>(internal_forces::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_characteristics(std::make_shared<internal_forces::muscles::Characteristics>
                      (characteristics)),
    m_state(std::make_shared<internal_forces::muscles::State>(dynamicState))
{

}

internal_forces::muscles::Muscle::Muscle(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &position,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers) :
    internal_forces::Compound (name, pathModifiers),
    m_position(std::make_shared<internal_forces::muscles::MuscleGeometry>(position)),
    m_type(std::make_shared<internal_forces::muscles::MUSCLE_TYPE>(internal_forces::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_characteristics(std::make_shared<internal_forces::muscles::Characteristics>
                      (characteristics)),
    m_state(std::make_shared<internal_forces::muscles::State>())
{

}

internal_forces::muscles::Muscle::Muscle(const internal_forces::muscles::Muscle &other) :
    internal_forces::Compound (other),
    m_position(other.m_position),
    m_type(other.m_type),
    m_characteristics(other.m_characteristics),
    m_state(other.m_state)
{

}

internal_forces::muscles::Muscle::Muscle(const std::shared_ptr<internal_forces::muscles::Muscle>
                                other) :
    internal_forces::Compound (other),
    m_position(other->m_position),
    m_type(other->m_type),
    m_characteristics(other->m_characteristics),
    m_state(other->m_state)
{

}

internal_forces::muscles::Muscle::Muscle(const utils::String& name,
                                const internal_forces::muscles::MuscleGeometry& g,
                                const internal_forces::muscles::Characteristics& c,
                                const internal_forces::PathModifiers &pathModifiers,
                                const internal_forces::muscles::State& emg) :
    internal_forces::Compound(name,pathModifiers),
    m_position(std::make_shared<internal_forces::muscles::MuscleGeometry>(g)),
    m_type(std::make_shared<internal_forces::muscles::MUSCLE_TYPE>(internal_forces::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_characteristics(std::make_shared<internal_forces::muscles::Characteristics>(c)),
    m_state(std::make_shared<internal_forces::muscles::State>())
{
    setState(emg);

    utils::Error::check(pathModifiers.nbWraps() <= 1,
                                "Multiple wrapping objects is not implemented yet");
}

internal_forces::muscles::Muscle::~Muscle()
{
    //dtor
}

void internal_forces::muscles::Muscle::DeepCopy(const internal_forces::muscles::Muscle &other)
{
    this->internal_forces::Compound::DeepCopy(other);
    *m_position = other.m_position->DeepCopy();
    *m_type = *other.m_type;
    *m_characteristics = other.m_characteristics->DeepCopy();
    *m_state = other.m_state->DeepCopy();
}

internal_forces::muscles::MUSCLE_TYPE internal_forces::muscles::Muscle::type() const
{
    return *m_type;
}

void internal_forces::muscles::Muscle::setType()
{
    *m_type = internal_forces::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE;
}

void internal_forces::muscles::Muscle::updateOrientations(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates &Q)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(updatedModel, *m_characteristics, *m_pathChanger, &Q, nullptr);
}
void internal_forces::muscles::Muscle::updateOrientations(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(updatedModel, *m_characteristics, *m_pathChanger, &Q, &Qdot);
}
void internal_forces::muscles::Muscle::updateOrientations(
    std::vector<utils::Vector3d>& musclePointsInGlobal,
    utils::Matrix &jacoPointsInGlobal)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(musclePointsInGlobal,jacoPointsInGlobal,*m_characteristics,nullptr);
}
void internal_forces::muscles::Muscle::updateOrientations(
    std::vector<utils::Vector3d>& musclePointsInGlobal,
    utils::Matrix &jacoPointsInGlobal,
    const rigidbody::GeneralizedVelocity &Qdot)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(musclePointsInGlobal,jacoPointsInGlobal,*m_characteristics,&Qdot);
}

void internal_forces::muscles::Muscle::setPosition(
    const internal_forces::muscles::MuscleGeometry &positions)
{
    *m_position = positions;
}
const internal_forces::muscles::MuscleGeometry &internal_forces::muscles::Muscle::position() const
{
    return *m_position;
}

const utils::Scalar& internal_forces::muscles::Muscle::length(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateMuscleParameters)
{
    if (updateMuscleParameters) m_position->updateKinematics(updatedModel, *m_characteristics, *m_pathChanger, &Q, nullptr);
    return position().length();
}

const utils::Scalar& internal_forces::muscles::Muscle::musculoTendonLength(
    rigidbody::Joints &updatedModel,
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateMuscleParameters)
{
    if (updateMuscleParameters) m_position->updateKinematics(updatedModel, *m_characteristics, *m_pathChanger, &Q, nullptr);
    return position().musculoTendonLength();
}

const utils::Scalar& internal_forces::muscles::Muscle::velocity(
    rigidbody::Joints &updatedModel,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateMuscleParameters)
{
    if (updateMuscleParameters) m_position->updateKinematics(updatedModel, *m_characteristics, *m_pathChanger, &Q, &Qdot);
    return m_position->velocity();
}

const utils::Scalar& internal_forces::muscles::Muscle::activationDot(
    const internal_forces::muscles::State& state,
    bool alreadyNormalized) const
{
    std::shared_ptr<internal_forces::muscles::StateDynamics> state_copy =
        std::dynamic_pointer_cast<internal_forces::muscles::StateDynamics>(m_state);
    utils::Error::check(
        state_copy != nullptr,
        "The muscle " + name() + " is not a dynamic muscle");
    return state_copy->timeDerivativeActivation(state, characteristics(), alreadyNormalized);
}

void internal_forces::muscles::Muscle::computeForce(const internal_forces::muscles::State &emg)
{
    *m_force = getForceFromActivation(emg);
}

const std::vector<utils::Vector3d>&
internal_forces::muscles::Muscle::musclesPointsInGlobal(
    rigidbody::Joints &updatedModel,
    const rigidbody::GeneralizedCoordinates &Q, 
    bool updateMuscleParameters)
{
    if (updateMuscleParameters) m_position->updateKinematics(updatedModel, *m_characteristics, *m_pathChanger, &Q, nullptr);
    return musclesPointsInGlobal();
}

const std::vector<utils::Vector3d>
&internal_forces::muscles::Muscle::musclesPointsInGlobal() const
{
    return m_position->pointsInGlobal();
}

void internal_forces::muscles::Muscle::setForceIsoMax(
    const utils::Scalar& forceMax)
{
    m_characteristics->setForceIsoMax(forceMax);
}

void internal_forces::muscles::Muscle::setCharacteristics(
    const internal_forces::muscles::Characteristics &characteristics)
{
    *m_characteristics = characteristics;
}

internal_forces::muscles::Characteristics& internal_forces::muscles::Muscle::characteristics() const
{
    return *m_characteristics;
}

// Get and set
void internal_forces::muscles::Muscle::setState(
    const internal_forces::muscles::State &emg)
{
    if (emg.type() == internal_forces::muscles::STATE_TYPE::BUCHANAN) {
        m_state = std::make_shared<internal_forces::muscles::StateDynamicsBuchanan>
                  (internal_forces::muscles::StateDynamicsBuchanan());
    } else if (emg.type() == internal_forces::muscles::STATE_TYPE::DE_GROOTE) {
        m_state = std::make_shared<internal_forces::muscles::StateDynamicsDeGroote>
                  (internal_forces::muscles::StateDynamicsDeGroote());
    } else if (emg.type() == internal_forces::muscles::STATE_TYPE::DYNAMIC) {
        m_state = std::make_shared<internal_forces::muscles::StateDynamics>
                  (internal_forces::muscles::StateDynamics());
    } else {
        utils::Error::raise(utils::String(
                                        internal_forces::muscles::STATE_TYPE_toStr(
                                            emg.type())) + " is not a valid type for setState");
    }
    *m_state = emg;
}
const internal_forces::muscles::State& internal_forces::muscles::Muscle::state() const
{
    return *m_state;
}
internal_forces::muscles::State& internal_forces::muscles::Muscle::state()
{
    return *m_state;
}
