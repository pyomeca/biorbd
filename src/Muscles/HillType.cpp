#define BIORBD_API_EXPORTS
#include "Muscles/HillType.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "Muscles/Characteristics.h"
#include "Muscles/Geometry.h"
#include "Muscles/State.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;

muscles::HillType::HillType() :
    muscles::Muscle(),
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
    m_cste_damping(std::make_shared<utils::Scalar>(0.1)),
    m_cste_maxShorteningSpeed(std::make_shared<utils::Scalar>(10.0))
{
    setType();
}

muscles::HillType::HillType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics) :
    muscles::Muscle(name,geometry,characteristics),
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
    m_cste_damping(std::make_shared<utils::Scalar>(0.1)),
    m_cste_maxShorteningSpeed(std::make_shared<utils::Scalar>(10.0))
{
    setType();
}

muscles::HillType::HillType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::State& emg) :
    muscles::Muscle(name,geometry,characteristics, emg),
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
    m_cste_damping(std::make_shared<utils::Scalar>(0.1)),
    m_cste_maxShorteningSpeed(std::make_shared<utils::Scalar>(10.0))
{
    setType();
}

muscles::HillType::HillType(
    const utils::String &name,
    const muscles::Geometry &geometry,
    const muscles::Characteristics &characteristics,
    const muscles::PathModifiers &pathModifiers) :
    muscles::Muscle(name,geometry,characteristics,pathModifiers),
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
    m_cste_damping(std::make_shared<utils::Scalar>(0.1)),
    m_cste_maxShorteningSpeed(std::make_shared<utils::Scalar>(10.0))
{
    setType();
}
muscles::HillType::HillType(
    const utils::String& name,
    const muscles::Geometry& geometry,
    const muscles::Characteristics& characteristics,
    const muscles::PathModifiers &pathModifiers,
    const muscles::State& state) :
    muscles::Muscle(name,geometry,characteristics,pathModifiers,state),
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
    m_cste_damping(std::make_shared<utils::Scalar>(0.1)),
    m_cste_maxShorteningSpeed(std::make_shared<utils::Scalar>(10.0))
{
    setType();
}

muscles::HillType::HillType(const muscles::Muscle &other) :
    muscles::Muscle (other)
{
    const muscles::HillType & m_tp(
        dynamic_cast<const muscles::HillType &>(other));
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
    m_cste_maxShorteningSpeed = m_tp.m_cste_maxShorteningSpeed;
}

muscles::HillType::HillType(
    const std::shared_ptr<muscles::Muscle> other) :
    muscles::Muscle (other)
{
    const std::shared_ptr<muscles::HillType> m_tp(
        std::dynamic_pointer_cast<muscles::HillType>(other));
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
    m_cste_maxShorteningSpeed = m_tp->m_cste_maxShorteningSpeed;
}

muscles::HillType muscles::HillType::DeepCopy() const
{
    muscles::HillType copy;
    copy.DeepCopy(*this);
    return copy;
}

void muscles::HillType::DeepCopy(const muscles::HillType &other)
{
    muscles::Muscle::DeepCopy(other);
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
    *m_cste_maxShorteningSpeed = *other.m_cste_maxShorteningSpeed;
}

const utils::Scalar& muscles::HillType::force(
    const muscles::State& emg)
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

const utils::Scalar& muscles::HillType::force(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    const muscles::State &emg,
    int updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = 2;
#endif
    // Update the configuration
    if (updateKin == 1) {
        updateOrientations(model,Q,Qdot,false);
    } else if (updateKin == 2) {
        updateOrientations(model,Q,Qdot,2);
    } else {
        utils::Error::check(updateKin == 0,
                                    "Wrong level of update in force function");
    }

    // Computation
    return force(emg);
}

const utils::Scalar& muscles::HillType::force(
    rigidbody::Joints &,
    const rigidbody::GeneralizedCoordinates &,
    const muscles::State &,
    int)
{
    utils::Error::raise("Hill type needs velocity");
#ifdef _WIN32
    return *m_force; // Will never reach here
#endif
}

const utils::Scalar& muscles::HillType::FlCE(
    const muscles::State &EMG)
{
    computeFlCE(EMG);
    return *m_FlCE;
}

const utils::Scalar& muscles::HillType::FlPE()
{
    computeFlPE();
    return *m_FlPE;
}

const utils::Scalar& muscles::HillType::FvCE()
{
    computeFvCE();
    return *m_FvCE;
}

const utils::Scalar& muscles::HillType::damping()
{
    computeDamping();
    return *m_damping;
}

void muscles::HillType::setType()
{
    *m_type = muscles::MUSCLE_TYPE::HILL;
}

void muscles::HillType::computeDamping()
{


#ifdef BIORBD_USE_CASADI_MATH
    *m_damping = IF_ELSE_NAMESPACE::if_else_zero(
                  IF_ELSE_NAMESPACE::gt(position().velocity(), 0),
                ((position().velocity() / (characteristics().optimalLength() * *m_cste_maxShorteningSpeed))
                * *m_cste_damping));

#else
    if (position().velocity() > 0) {
        *m_damping = (position().velocity()
                     /
                     (characteristics().optimalLength() * *m_cste_maxShorteningSpeed)) * *m_cste_damping;
    } else {
        *m_damping = 0;
    }
#endif

}

void muscles::HillType::computeFlCE(const muscles::State& emg)
{
    *m_FlCE = exp( -pow(( position().length() /
                          m_characteristics->optimalLength() / (*m_cste_FlCE_1*
                                  (1-emg.activation())+1) -1 ), 2)
                   /
                   *m_cste_FlCE_2   );
}

void muscles::HillType::computeFvCE()
{
    // The relation is different if velocity< 0  or > 0
    utils::Scalar v = m_position->velocity();
#ifdef BIORBD_USE_CASADI_MATH
    *m_FvCE = IF_ELSE_NAMESPACE::if_else(
                  IF_ELSE_NAMESPACE::le(v, 0),
                  (1.0-std::fabs(v) / *m_cste_maxShorteningSpeed) /
                  (1.0+std::fabs(v) / *m_cste_maxShorteningSpeed / *m_cste_FvCE_1),
                  (1.0-1.33*v / *m_cste_maxShorteningSpeed / *m_cste_FvCE_2) /
                  (1-v / *m_cste_maxShorteningSpeed / *m_cste_FvCE_2)
              );
#else
    if (v<=0)
        *m_FvCE = (1-fabs(v) / *m_cste_maxShorteningSpeed) /
                  (1+fabs(v) / *m_cste_maxShorteningSpeed / *m_cste_FvCE_1);
    else
        *m_FvCE = (1-1.33*v / *m_cste_maxShorteningSpeed / *m_cste_FvCE_2) /
                  (1-v / *m_cste_maxShorteningSpeed / *m_cste_FvCE_2);
#endif
}

void muscles::HillType::computeFlPE()
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

utils::Scalar muscles::HillType::getForceFromActivation(
    const muscles::State &emg)
{
    utils::Scalar cosAngle = cos(characteristics().pennationAngle());
    utils::Scalar damping_param;
    if (characteristics().useDamping()==true) {
         damping_param = *m_damping;
    } else {
        damping_param  = 0;
    }
   // utils::Scalar force = 0;
/*#ifdef BIORBD_USE_CASADI_MATH
    force = IF_ELSE_NAMESPACE::if_else(
                  IF_ELSE_NAMESPACE::gt(characteristics().useDamping(), 0),
                  characteristics().forceIsoMax() * (emg.activation() * *m_FlCE * *m_FvCE + *m_FlPE + *m_damping) * cosAngle
                                        ,
                  characteristics().forceIsoMax() * (emg.activation() *  *m_FlCE * *m_FvCE + *m_FlPE) * cosAngle);
#else
    if (characteristics().useDamping() != 0) {
        force = characteristics().forceIsoMax() * (emg.activation() * *m_FlCE * *m_FvCE + *m_FlPE) * cosAngle;
    } else {
        force = characteristics().forceIsoMax() * (emg.activation() * *m_FlCE * *m_FvCE + *m_FlPE + *m_damping) * cosAngle;
    }
#endif
    // Fonction qui permet de modifier la facon dont la multiplication est faite dans computeForce(EMG)
    //return characteristics().forceIsoMax() * (emg.activation() * *m_FlCE * *m_FvCE + *m_FlPE + *m_damping) * cosAngle;*/
    return characteristics().forceIsoMax() * (emg.activation() * *m_FlCE * *m_FvCE + *m_FlPE + damping_param) * cosAngle;
}

void muscles::HillType::normalizeEmg(
    muscles::State& emg)
{
    emg.normalizeExcitation(characteristics().stateMax());
}
