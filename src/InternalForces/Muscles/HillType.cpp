#define BIORBD_API_EXPORTS

#include "Utils/Error.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "InternalForces/Muscles/Characteristics.h"
#include "InternalForces/Muscles/MuscleGeometry.h"
#include "InternalForces/Muscles/State.h"
#include "InternalForces/Muscles/HillType.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;
internal_forces::muscles::HillType::HillType() :
    internal_forces::muscles::Muscle(),
    m_damping(std::make_shared<utils::Scalar>()),
    m_FlCE(std::make_shared<utils::Scalar>()),
    m_FlPE(std::make_shared<utils::Scalar>()),
    m_FvCE(std::make_shared<utils::Scalar>()),
    m_cste_FlCE_1(std::make_shared<utils::Scalar>(0.15)),
    m_cste_FlCE_2(std::make_shared<utils::Scalar>(0.45)),
    m_cste_FvCE_1(std::make_shared<utils::Scalar>(1)),
    m_cste_FvCE_2(std::make_shared<utils::Scalar>(-.33/2 * *m_cste_FvCE_1/
                  (1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<utils::Scalar>(10.0)),
    m_cste_FlPE_2(std::make_shared<utils::Scalar>(5.0)),
    m_cste_eccentricForceMultiplier(std::make_shared<utils::Scalar>(1.8)),
    m_cste_damping(std::make_shared<utils::Scalar>(0.1))
{
    setType();
}

internal_forces::muscles::HillType::HillType(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics) :
    internal_forces::muscles::Muscle(name,geometry,characteristics),
    m_damping(std::make_shared<utils::Scalar>()),
    m_FlCE(std::make_shared<utils::Scalar>()),
    m_FlPE(std::make_shared<utils::Scalar>()),
    m_FvCE(std::make_shared<utils::Scalar>()),
    m_cste_FlCE_1(std::make_shared<utils::Scalar>(0.15)),
    m_cste_FlCE_2(std::make_shared<utils::Scalar>(0.45)),
    m_cste_FvCE_1(std::make_shared<utils::Scalar>(1)),
    m_cste_FvCE_2(std::make_shared<utils::Scalar>(-.33/2 * *m_cste_FvCE_1/
                  (1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<utils::Scalar>(10.0)),
    m_cste_FlPE_2(std::make_shared<utils::Scalar>(5.0)),
    m_cste_eccentricForceMultiplier(std::make_shared<utils::Scalar>(1.8)),
    m_cste_damping(std::make_shared<utils::Scalar>(0.1))
{
    setType();
}

internal_forces::muscles::HillType::HillType(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::muscles::State& emg) :
    internal_forces::muscles::Muscle(name,geometry,characteristics, emg),
    m_damping(std::make_shared<utils::Scalar>()),
    m_FlCE(std::make_shared<utils::Scalar>()),
    m_FlPE(std::make_shared<utils::Scalar>()),
    m_FvCE(std::make_shared<utils::Scalar>()),
    m_cste_FlCE_1(std::make_shared<utils::Scalar>(0.15)),
    m_cste_FlCE_2(std::make_shared<utils::Scalar>(0.45)),
    m_cste_FvCE_1(std::make_shared<utils::Scalar>(1)),
    m_cste_FvCE_2(std::make_shared<utils::Scalar>(-.33/2 * *m_cste_FvCE_1/
                  (1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<utils::Scalar>(10.0)),
    m_cste_FlPE_2(std::make_shared<utils::Scalar>(5.0)),
    m_cste_eccentricForceMultiplier(std::make_shared<utils::Scalar>(1.8)),
    m_cste_damping(std::make_shared<utils::Scalar>(0.1))
{
    setType();
}

internal_forces::muscles::HillType::HillType(
    const utils::String &name,
    const internal_forces::muscles::MuscleGeometry &geometry,
    const internal_forces::muscles::Characteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers) :
    internal_forces::muscles::Muscle(name,geometry,characteristics,pathModifiers),
    m_damping(std::make_shared<utils::Scalar>()),
    m_FlCE(std::make_shared<utils::Scalar>()),
    m_FlPE(std::make_shared<utils::Scalar>()),
    m_FvCE(std::make_shared<utils::Scalar>()),
    m_cste_FlCE_1(std::make_shared<utils::Scalar>(0.15)),
    m_cste_FlCE_2(std::make_shared<utils::Scalar>(0.45)),
    m_cste_FvCE_1(std::make_shared<utils::Scalar>(1)),
    m_cste_FvCE_2(std::make_shared<utils::Scalar>(-.33/2 * *m_cste_FvCE_1/
                  (1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<utils::Scalar>(10.0)),
    m_cste_FlPE_2(std::make_shared<utils::Scalar>(5.0)),
    m_cste_eccentricForceMultiplier(std::make_shared<utils::Scalar>(1.8)),
    m_cste_damping(std::make_shared<utils::Scalar>(0.1))
{
    setType();
}
internal_forces::muscles::HillType::HillType(
    const utils::String& name,
    const internal_forces::muscles::MuscleGeometry& geometry,
    const internal_forces::muscles::Characteristics& characteristics,
    const internal_forces::PathModifiers &pathModifiers,
    const internal_forces::muscles::State& state) :
    internal_forces::muscles::Muscle(name,geometry,characteristics,pathModifiers,state),
    m_damping(std::make_shared<utils::Scalar>()),
    m_FlCE(std::make_shared<utils::Scalar>()),
    m_FlPE(std::make_shared<utils::Scalar>()),
    m_FvCE(std::make_shared<utils::Scalar>()),
    m_cste_FlCE_1(std::make_shared<utils::Scalar>(0.15)),
    m_cste_FlCE_2(std::make_shared<utils::Scalar>(0.45)),
    m_cste_FvCE_1(std::make_shared<utils::Scalar>(1)),
    m_cste_FvCE_2(std::make_shared<utils::Scalar>(-.33/2 * *m_cste_FvCE_1/
                  (1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<utils::Scalar>(10.0)),
    m_cste_FlPE_2(std::make_shared<utils::Scalar>(5.0)),
    m_cste_eccentricForceMultiplier(std::make_shared<utils::Scalar>(1.8)),
    m_cste_damping(std::make_shared<utils::Scalar>(0.1))
{
    setType();
}

internal_forces::muscles::HillType::HillType(const internal_forces::muscles::Muscle &other) :
    internal_forces::muscles::Muscle (other)
{
    const internal_forces::muscles::HillType & m_tp(
        dynamic_cast<const internal_forces::muscles::HillType &>(other));
    m_damping = m_tp.m_damping;
    m_FlCE = m_tp.m_FlCE;
    m_FlPE = m_tp.m_FlPE;
    m_FvCE = m_tp.m_FvCE;
    m_cste_FlCE_1 = m_tp.m_cste_FlCE_1;
    m_cste_FlCE_2 = m_tp.m_cste_FlCE_2;
    m_cste_FvCE_1 = m_tp.m_cste_FvCE_1;
    m_cste_FvCE_2 = m_tp.m_cste_FvCE_2;
    m_cste_FlPE_1 = m_tp.m_cste_FlPE_1;
    m_cste_FlPE_2 = m_tp.m_cste_FlPE_2;
    m_cste_eccentricForceMultiplier = m_tp.m_cste_eccentricForceMultiplier;
    m_cste_damping = m_tp.m_cste_damping;
}

internal_forces::muscles::HillType::HillType(
    const std::shared_ptr<internal_forces::muscles::Muscle> other) :
    internal_forces::muscles::Muscle (other)
{
    const std::shared_ptr<internal_forces::muscles::HillType> m_tp(
        std::dynamic_pointer_cast<internal_forces::muscles::HillType>(other));
    utils::Error::check(m_tp != nullptr, "Muscle must be of a Hill Type");
    m_damping = m_tp->m_damping;
    m_FlCE = m_tp->m_FlCE;
    m_FlPE = m_tp->m_FlPE;
    m_FvCE = m_tp->m_FvCE;
    m_cste_FlCE_1 = m_tp->m_cste_FlCE_1;
    m_cste_FlCE_2 = m_tp->m_cste_FlCE_2;
    m_cste_FvCE_1 = m_tp->m_cste_FvCE_1;
    m_cste_FvCE_2 = m_tp->m_cste_FvCE_2;
    m_cste_FlPE_1 = m_tp->m_cste_FlPE_1;
    m_cste_FlPE_2 = m_tp->m_cste_FlPE_2;
    m_cste_eccentricForceMultiplier = m_tp->m_cste_eccentricForceMultiplier;
    m_cste_damping = m_tp->m_cste_damping;
}

internal_forces::muscles::HillType internal_forces::muscles::HillType::DeepCopy() const
{
    internal_forces::muscles::HillType copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::muscles::HillType::DeepCopy(const internal_forces::muscles::HillType &other)
{
    internal_forces::muscles::Muscle::DeepCopy(other);
    *m_damping = *other.m_damping;
    *m_FlCE = *other.m_FlCE;
    *m_FlPE = *other.m_FlPE;
    *m_FvCE = *other.m_FvCE;
    *m_cste_FlCE_1 = *other.m_cste_FlCE_1;
    *m_cste_FlCE_2 = *other.m_cste_FlCE_2;
    *m_cste_FvCE_1 = *other.m_cste_FvCE_1;
    *m_cste_FvCE_2 = *other.m_cste_FvCE_2;
    *m_cste_FlPE_1 = *other.m_cste_FlPE_1;
    *m_cste_FlPE_2 = *other.m_cste_FlPE_2;
    *m_cste_eccentricForceMultiplier = *other.m_cste_eccentricForceMultiplier;
    *m_cste_damping = *other.m_cste_damping;
}

const utils::Scalar& internal_forces::muscles::HillType::force(
    const internal_forces::muscles::State& emg)
{
    // Compute the forces of each element
    computeFvCE();
    computeFlCE(emg);
    computeFlPE();
    computeDamping();

    // Combine the forces
    computeForce(emg);
    return *m_force;
}

const utils::Scalar& internal_forces::muscles::HillType::force(
    rigidbody::Joints &updatedModel,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const internal_forces::muscles::State &emg,
    bool updateMuscleParameters)
{
    // Update the configuration
    if (updateMuscleParameters) updateOrientations(updatedModel, Q, Qdot);

    // Computation
    return force(emg);
}

const utils::Scalar& internal_forces::muscles::HillType::force(
    rigidbody::Joints &,
    const rigidbody::GeneralizedCoordinates &,
    const internal_forces::muscles::State &,
    bool)
{
    utils::Error::raise("Hill type needs velocity");
#ifdef _WIN32
    return *m_force; // Will never reach here
#endif
}

const utils::Scalar& internal_forces::muscles::HillType::FlCE(
    const internal_forces::muscles::State &EMG)
{
    computeFlCE(EMG);
    return *m_FlCE;
}

const utils::Scalar& internal_forces::muscles::HillType::FlPE()
{
    computeFlPE();
    return *m_FlPE;
}

const utils::Scalar& internal_forces::muscles::HillType::FvCE()
{
    computeFvCE();
    return *m_FvCE;
}

const utils::Scalar& internal_forces::muscles::HillType::damping()
{
    computeDamping();
    return *m_damping;
}

void internal_forces::muscles::HillType::setType()
{
    *m_type = internal_forces::muscles::MUSCLE_TYPE::HILL;
}

void internal_forces::muscles::HillType::computeDamping()
{


#ifdef BIORBD_USE_CASADI_MATH
    *m_damping = IF_ELSE_NAMESPACE::if_else_zero(
                  IF_ELSE_NAMESPACE::gt(position().velocity(), 0),
                ((position().velocity() / (characteristics().optimalLength() * this->characteristics().maxShorteningSpeed()))
                * *m_cste_damping));

#else
    if (position().velocity() > 0) {
        *m_damping = (position().velocity()
                     /
                     (characteristics().optimalLength() * this->characteristics().maxShorteningSpeed())) * *m_cste_damping;
    } else {
        *m_damping = 0;
    }
#endif

}

void internal_forces::muscles::HillType::computeFlCE(const internal_forces::muscles::State& emg)
{
    *m_FlCE = exp( -pow(( position().length()/
                          m_characteristics->optimalLength() / (*m_cste_FlCE_1*
                                  (1-emg.activation())+1) -1 ), 2)
                   /
                   *m_cste_FlCE_2   );
}

void internal_forces::muscles::HillType::computeFvCE()
{
    // The relation is different if velocity< 0  or > 0
    utils::Scalar v = m_position->velocity();
#ifdef BIORBD_USE_CASADI_MATH
    *m_FvCE = IF_ELSE_NAMESPACE::if_else(
                  IF_ELSE_NAMESPACE::le(v, 0),
                  (1.0-std::fabs(v) / this->characteristics().maxShorteningSpeed()) /
                  (1.0+std::fabs(v) / this->characteristics().maxShorteningSpeed() / *m_cste_FvCE_1),
                  (1.0-1.33*v / this->characteristics().maxShorteningSpeed() / *m_cste_FvCE_2) /
                  (1-v / this->characteristics().maxShorteningSpeed() / *m_cste_FvCE_2)
              );
#else
    if (v<=0)
        *m_FvCE = (1-fabs(v) / this->characteristics().maxShorteningSpeed()) /
                  (1+fabs(v) / this->characteristics().maxShorteningSpeed() / *m_cste_FvCE_1);
    else
        *m_FvCE = (1-1.33*v / this->characteristics().maxShorteningSpeed() / *m_cste_FvCE_2) /
                  (1-v / this->characteristics().maxShorteningSpeed() / *m_cste_FvCE_2);
#endif
}

void internal_forces::muscles::HillType::computeFlPE()
{

#ifdef BIORBD_USE_CASADI_MATH
    *m_FlPE = IF_ELSE_NAMESPACE::if_else_zero(
                  IF_ELSE_NAMESPACE::gt(position().length(), 0),
                  exp(*m_cste_FlPE_1*(position().length()/characteristics().optimalLength()-1) -
                      *m_cste_FlPE_2));
#else
    if (position().length() > 0) {
        *m_FlPE = exp(*m_cste_FlPE_1*
                      (position().length()/characteristics().optimalLength()-1) - *m_cste_FlPE_2);
    } else {
        *m_FlPE = 0;
    }
#endif
}

utils::Scalar internal_forces::muscles::HillType::getForceFromActivation(
    const internal_forces::muscles::State &emg)
{
    utils::Scalar cosAngle = cos(characteristics().pennationAngle());
    utils::Scalar damping_param;
    if (characteristics().useDamping()){
         damping_param = *m_damping;
    } else {
        damping_param  = 0;
    }
    return characteristics().forceIsoMax() * (emg.activation() * *m_FlCE * *m_FvCE + *m_FlPE + damping_param) * cosAngle;
}

void internal_forces::muscles::HillType::normalizeEmg(
    internal_forces::muscles::State& emg)
{
    emg.normalizeExcitation(characteristics().stateMax());
}
