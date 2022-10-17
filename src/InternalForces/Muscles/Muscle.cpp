#define BIORBD_API_EXPORTS
#include "InternalForces/Muscles/Muscle.h"

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
#include "InternalForces/Muscles/Geometry.h"

using namespace BIORBD_NAMESPACE;
internalforce::muscles::Muscle::Muscle() :
    internalforce::Compound(),
    m_type(std::make_shared<internalforce::muscles::MUSCLE_TYPE>(muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_position(std::make_shared<internalforce::muscles::Geometry>()),
    m_characteristics(std::make_shared<internalforce::muscles::Characteristics>()),
    m_state(std::make_shared<internalforce::muscles::State>())
{

}

internalforce::muscles::Muscle::Muscle(
    const utils::String & name,
    const internalforce::muscles::Geometry & position,
    const internalforce::muscles::Characteristics &characteristics) :
    internalforce::Compound (name),
    m_type(std::make_shared<internalforce::muscles::MUSCLE_TYPE>(muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_position(std::make_shared<internalforce::muscles::Geometry>(position)),
    m_characteristics(std::make_shared<internalforce::muscles::Characteristics>
                      (characteristics)),
    m_state(std::make_shared<internalforce::muscles::State>())
{

}

internalforce::muscles::Muscle::Muscle(
    const utils::String &name,
    const internalforce::muscles::Geometry &position,
    const internalforce::muscles::Characteristics &characteristics,
    const internalforce::muscles::State &dynamicState) :
    internalforce::Compound (name),
    m_type(std::make_shared<internalforce::muscles::MUSCLE_TYPE>(muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_position(std::make_shared<internalforce::muscles::Geometry>(position)),
    m_characteristics(std::make_shared<internalforce::muscles::Characteristics>
                      (characteristics)),
    m_state(std::make_shared<internalforce::muscles::State>(dynamicState))
{

}

internalforce::muscles::Muscle::Muscle(
    const utils::String &name,
    const internalforce::muscles::Geometry &position,
    const internalforce::muscles::Characteristics &characteristics,
    const internalforce::PathModifiers &pathModifiers) :
    internalforce::Compound (name, pathModifiers),
    m_type(std::make_shared<internalforce::muscles::MUSCLE_TYPE>(muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_position(std::make_shared<internalforce::muscles::Geometry>(position)),
    m_characteristics(std::make_shared<internalforce::muscles::Characteristics>
                      (characteristics)),
    m_state(std::make_shared<internalforce::muscles::State>())
{

}

internalforce::muscles::Muscle::Muscle(const internalforce::muscles::Muscle &other) :
    internalforce::Compound (other),
    m_type(other.m_type),
    m_position(other.m_position),
    m_characteristics(other.m_characteristics),
    m_state(other.m_state)
{

}

internalforce::muscles::Muscle::Muscle(const std::shared_ptr<internalforce::muscles::Muscle>
                                other) :
    internalforce::Compound (other),
    m_type(other->m_type),
    m_position(other->m_position),
    m_characteristics(other->m_characteristics),
    m_state(other->m_state)
{

}

internalforce::muscles::Muscle::Muscle(const utils::String& name,
                                const internalforce::muscles::Geometry& g,
                                const internalforce::muscles::Characteristics& c,
                                const internalforce::PathModifiers &pathModifiers,
                                const internalforce::muscles::State& emg) :
    internalforce::Compound(name,pathModifiers),
    m_type(std::make_shared<internalforce::muscles::MUSCLE_TYPE>(muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_position(std::make_shared<internalforce::muscles::Geometry>(g)),
    m_characteristics(std::make_shared<internalforce::muscles::Characteristics>(c)),
    m_state(std::make_shared<internalforce::muscles::State>())
{
    setState(emg);

    utils::Error::check(pathModifiers.nbWraps() <= 1,
                                "Multiple wrapping objects is not implemented yet");
}

internalforce::muscles::Muscle::~Muscle()
{
    //dtor
}

internalforce::muscles::MUSCLE_TYPE internalforce::muscles::type() const
{
    return *m_type;
}

void internalforce::muscles::setType()
{
    *m_type = internalforce::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE;
}

void internalforce::muscles::Muscle::DeepCopy(const internalforce::muscles::Muscle &other)
{
    this->internalforce::Compound::DeepCopy(other);
    *m_position = other.m_position->DeepCopy();
    *m_characteristics = other.m_characteristics->DeepCopy();
    *m_state = other.m_state->DeepCopy();
}

void internalforce::muscles::Muscle::updateOrientations(
    rigidbody::Joints& model,
    const rigidbody::GeneralizedCoordinates &Q,
    int updateKin)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(model,*m_characteristics,*m_pathChanger,&Q,nullptr,
                                 updateKin);
}
void internalforce::muscles::Muscle::updateOrientations(
    rigidbody::Joints& model,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    int updateKin)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(
        model,*m_characteristics,*m_pathChanger,&Q,&Qdot, updateKin);
}
void internalforce::muscles::Muscle::updateOrientations(
    std::vector<utils::Vector3d>& musclePointsInGlobal,
    utils::Matrix &jacoPointsInGlobal)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(
                musclePointsInGlobal,jacoPointsInGlobal, *m_characteristics,nullptr);
}
void internalforce::muscles::Muscle::updateOrientations(
    std::vector<utils::Vector3d>& musclePointsInGlobal,
    utils::Matrix &jacoPointsInGlobal,
    const rigidbody::GeneralizedVelocity &Qdot)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(
                musclePointsInGlobal,jacoPointsInGlobal, *m_characteristics,&Qdot);
}

void internalforce::muscles::Muscle::setPosition(
    const internalforce::muscles::Geometry &positions)
{
    *m_position = positions;
}
const internalforce::muscles::Geometry &internalforce::muscles::Muscle::position() const
{
    return *m_position;
}

const utils::Scalar& internalforce::muscles::Muscle::length(
    rigidbody::Joints& model,
    const rigidbody::GeneralizedCoordinates &Q,
    int updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = 2;
#endif
    if (updateKin != 0) {
        m_position->updateKinematics(
            model,*m_characteristics,*m_pathChanger,&Q,nullptr,updateKin);
    }

    return position().length();
}

const utils::Scalar& internalforce::muscles::Muscle::musculoTendonLength(
    rigidbody::Joints &m,
    const rigidbody::GeneralizedCoordinates &Q,
    int updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = 2;
#endif
    if (updateKin != 0) {
        m_position->updateKinematics(
            m,*m_characteristics,*m_pathChanger,&Q,nullptr,updateKin);
    }

    return m_position->musculoTendonLength();
}

const utils::Scalar& internalforce::muscles::Muscle::velocity(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        m_position->updateKinematics(
            model,*m_characteristics,*m_pathChanger,&Q,&Qdot);
    }

    return m_position->velocity();
}

const utils::Scalar& internalforce::muscles::Muscle::activationDot(
    const internalforce::muscles::State& state,
    bool alreadyNormalized) const
{
    std::shared_ptr<internalforce::muscles::StateDynamics> state_copy =
        std::dynamic_pointer_cast<internalforce::muscles::StateDynamics>(m_state);
    utils::Error::check(
        state_copy != nullptr,
        "The muscle " + name() + " is not a dynamic muscle");
    return state_copy->timeDerivativeActivation(
               state, characteristics(), alreadyNormalized);
}

void internalforce::muscles::Muscle::computeForce(const internalforce::muscles::State &emg)
{
    *m_force = getForceFromActivation(emg);
}

const std::vector<utils::Vector3d>&
internalforce::muscles::Muscle::musclesPointsInGlobal(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q)
{
    m_position->updateKinematics(
                model,*m_characteristics,*m_pathChanger,&Q,nullptr);

    return musclesPointsInGlobal();
}

const std::vector<utils::Vector3d>
&internalforce::muscles::Muscle::musclesPointsInGlobal() const
{
    return m_position->musclesPointsInGlobal();
}

void internalforce::muscles::Muscle::setForceIsoMax(
    const utils::Scalar& forceMax)
{
    m_characteristics->setForceIsoMax(forceMax);
}

void internalforce::muscles::Muscle::setCharacteristics(
    const internalforce::muscles::Characteristics &characteristics)
{
    *m_characteristics = characteristics;
}
const internalforce::muscles::Characteristics&
internalforce::muscles::Muscle::characteristics() const
{
    return *m_characteristics;
}

// Get and set
void internalforce::muscles::Muscle::setState(
    const internalforce::muscles::State &emg)
{
    if (emg.type() == internalforce::muscles::STATE_TYPE::BUCHANAN) {
        m_state = std::make_shared<internalforce::muscles::StateDynamicsBuchanan>
                  (internalforce::muscles::StateDynamicsBuchanan());
    } else if (emg.type() == internalforce::muscles::STATE_TYPE::DE_GROOTE) {
        m_state = std::make_shared<internalforce::muscles::StateDynamicsDeGroote>
                  (internalforce::muscles::StateDynamicsDeGroote());
    } else if (emg.type() == internalforce::muscles::STATE_TYPE::DYNAMIC) {
        m_state = std::make_shared<internalforce::muscles::StateDynamics>
                  (internalforce::muscles::StateDynamics());
    } else {
        utils::Error::raise(utils::String(
                                        internalforce::muscles::STATE_TYPE_toStr(
                                            emg.type())) + " is not a valid type for setState");
    }
    *m_state = emg;
}
const internalforce::muscles::State& internalforce::muscles::Muscle::state() const
{
    return *m_state;
}
internalforce::muscles::State& internalforce::muscles::Muscle::state()
{
    return *m_state;
}
