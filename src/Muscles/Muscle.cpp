#define BIORBD_API_EXPORTS
#include "Muscles/Muscle.h"

#include "Utils/Error.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "Muscles/PathModifiers.h"
#include "Muscles/StateDynamics.h"
#include "Muscles/StateDynamicsBuchanan.h"
#include "Muscles/StateDynamicsDeGroote.h"
#include "Muscles/Characteristics.h"
#include "Muscles/Geometry.h"

using namespace BIORBD_NAMESPACE;

muscles::Muscle::Muscle() :
    muscles::Compound(),
    m_position(std::make_shared<muscles::Geometry>()),
    m_characteristics(std::make_shared<muscles::Characteristics>()),
    m_state(std::make_shared<muscles::State>())
{

}

muscles::Muscle::Muscle(
    const utils::String & name,
    const muscles::Geometry & position,
    const muscles::Characteristics &characteristics) :
    muscles::Compound (name),
    m_position(std::make_shared<muscles::Geometry>(position)),
    m_characteristics(std::make_shared<muscles::Characteristics>
                      (characteristics)),
    m_state(std::make_shared<muscles::State>())
{

}

muscles::Muscle::Muscle(
    const utils::String &name,
    const muscles::Geometry &position,
    const muscles::Characteristics &characteristics,
    const muscles::State &dynamicState) :
    muscles::Compound (name),
    m_position(std::make_shared<muscles::Geometry>(position)),
    m_characteristics(std::make_shared<muscles::Characteristics>
                      (characteristics)),
    m_state(std::make_shared<muscles::State>(dynamicState))
{

}

muscles::Muscle::Muscle(
    const utils::String &name,
    const muscles::Geometry &position,
    const muscles::Characteristics &characteristics,
    const muscles::PathModifiers &pathModifiers) :
    muscles::Compound (name, pathModifiers),
    m_position(std::make_shared<muscles::Geometry>(position)),
    m_characteristics(std::make_shared<muscles::Characteristics>
                      (characteristics)),
    m_state(std::make_shared<muscles::State>())
{

}

muscles::Muscle::Muscle(const muscles::Muscle &other) :
    muscles::Compound (other),
    m_position(other.m_position),
    m_characteristics(other.m_characteristics),
    m_state(other.m_state)
{

}

muscles::Muscle::Muscle(const std::shared_ptr<muscles::Muscle>
                                other) :
    muscles::Compound (other),
    m_position(other->m_position),
    m_characteristics(other->m_characteristics),
    m_state(other->m_state)
{

}

muscles::Muscle::Muscle(const utils::String& name,
                                const muscles::Geometry& g,
                                const muscles::Characteristics& c,
                                const muscles::PathModifiers &pathModifiers,
                                const muscles::State& emg) :
    muscles::Compound(name,pathModifiers),
    m_position(std::make_shared<muscles::Geometry>(g)),
    m_characteristics(std::make_shared<muscles::Characteristics>(c)),
    m_state(std::make_shared<muscles::State>())
{
    setState(emg);

    utils::Error::check(pathModifiers.nbWraps() <= 1,
                                "Multiple wrapping objects is not implemented yet");
}

muscles::Muscle::~Muscle()
{
    //dtor
}

void muscles::Muscle::DeepCopy(const muscles::Muscle &other)
{
    this->muscles::Compound::DeepCopy(other);
    *m_position = other.m_position->DeepCopy();
    *m_characteristics = other.m_characteristics->DeepCopy();
    *m_state = other.m_state->DeepCopy();
}

void muscles::Muscle::updateOrientations(
    rigidbody::Joints& model,
    const rigidbody::GeneralizedCoordinates &Q,
    int updateKin)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(model,*m_characteristics,*m_pathChanger,&Q,nullptr,
                                 updateKin);
}
void muscles::Muscle::updateOrientations(
    rigidbody::Joints& model,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    int updateKin)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(model,*m_characteristics,*m_pathChanger,&Q,&Qdot,
                                 updateKin);
}
void muscles::Muscle::updateOrientations(
    std::vector<utils::Vector3d>& musclePointsInGlobal,
    utils::Matrix &jacoPointsInGlobal)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(musclePointsInGlobal,jacoPointsInGlobal,
                                 *m_characteristics,nullptr);
}
void muscles::Muscle::updateOrientations(
    std::vector<utils::Vector3d>& musclePointsInGlobal,
    utils::Matrix &jacoPointsInGlobal,
    const rigidbody::GeneralizedVelocity &Qdot)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(musclePointsInGlobal,jacoPointsInGlobal,
                                 *m_characteristics,&Qdot);
}

void muscles::Muscle::setPosition(
    const muscles::Geometry &positions)
{
    *m_position = positions;
}
const muscles::Geometry &muscles::Muscle::position() const
{
    return *m_position;
}

const utils::Scalar& muscles::Muscle::length(
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

const utils::Scalar& muscles::Muscle::musculoTendonLength(
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

const utils::Scalar& muscles::Muscle::velocity(
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

const utils::Scalar& muscles::Muscle::activationDot(
    const muscles::State& state,
    bool alreadyNormalized) const
{
    std::shared_ptr<muscles::StateDynamics> state_copy =
        std::dynamic_pointer_cast<muscles::StateDynamics>(m_state);
    utils::Error::check(
        state_copy != nullptr,
        "The muscle " + name() + " is not a dynamic muscle");
    return state_copy->timeDerivativeActivation(
               state, characteristics(), alreadyNormalized);
}

void muscles::Muscle::computeForce(const muscles::State &emg)
{
    *m_force = getForceFromActivation(emg);
}

const std::vector<utils::Vector3d>&
muscles::Muscle::musclesPointsInGlobal(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q)
{
    m_position->updateKinematics(model,*m_characteristics,*m_pathChanger,&Q,
                                 nullptr);

    return musclesPointsInGlobal();
}

const std::vector<utils::Vector3d>
&muscles::Muscle::musclesPointsInGlobal() const
{
    return m_position->musclesPointsInGlobal();
}

void muscles::Muscle::setForceIsoMax(
    const utils::Scalar& forceMax)
{
    m_characteristics->setForceIsoMax(forceMax);
}

void muscles::Muscle::setCharacteristics(
    const muscles::Characteristics &characteristics)
{
    *m_characteristics = characteristics;
}
const muscles::Characteristics&
muscles::Muscle::characteristics() const
{
    return *m_characteristics;
}

// Get and set
void muscles::Muscle::setState(
    const muscles::State &emg)
{
    if (emg.type() == muscles::STATE_TYPE::BUCHANAN) {
        m_state = std::make_shared<muscles::StateDynamicsBuchanan>
                  (muscles::StateDynamicsBuchanan());
    } else if (emg.type() == muscles::STATE_TYPE::DE_GROOTE_STATE) {
        m_state = std::make_shared<muscles::StateDynamicsDeGroote>
                  (muscles::StateDynamicsDeGroote());
    } else if (emg.type() == muscles::STATE_TYPE::DYNAMIC) {
        m_state = std::make_shared<muscles::StateDynamics>
                  (muscles::StateDynamics());
    } else {
        utils::Error::raise(utils::String(
                                        muscles::STATE_TYPE_toStr(
                                            emg.type())) + " is not a valid type for setState");
    }
    *m_state = emg;
}
const muscles::State& muscles::Muscle::state() const
{
    return *m_state;
}
muscles::State& muscles::Muscle::state()
{
    return *m_state;
}
