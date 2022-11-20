#define BIORBD_API_EXPORTS

#include "Utils/Error.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "InternalForces/PathModifiers.h"
#include "InternalForces/Compound.h"
#include "InternalForces/Geometry.h"
#include "InternalForces/Ligaments/LigamentsEnums.h"
#include "InternalForces/Ligaments/Characteristics.h"
#include "InternalForces/Ligaments/Ligament.h"

using namespace BIORBD_NAMESPACE;
internal_forces::ligaments::Ligament::Ligament() :
    m_compound(std::make_shared<internal_forces::Compound>()),
    m_position(std::make_shared<internal_forces::Geometry>()),
    m_type(std::make_shared<internal_forces::ligaments::LIGAMENT_TYPE>(internal_forces::ligaments::LIGAMENT_TYPE::NO_LIGAMENT_TYPE)),
    m_characteristics(std::make_shared<internal_forces::ligaments::Characteristics>())
{
    setType();

}

internal_forces::ligaments::Ligament::Ligament(
    const utils::String & name,
    const internal_forces::Geometry & position,
    const internal_forces::ligaments::Characteristics &characteristics) :
    m_compound(std::make_shared<internal_forces::Compound>(name)),
    m_position(std::make_shared<internal_forces::Geometry>(position)),
    m_type(std::make_shared<internal_forces::ligaments::LIGAMENT_TYPE>(internal_forces::ligaments::LIGAMENT_TYPE::NO_LIGAMENT_TYPE)),
    m_characteristics(std::make_shared<internal_forces::ligaments::Characteristics>
                      (characteristics))
{

}

internal_forces::ligaments::Ligament::Ligament(
    const utils::String &name,
    const internal_forces::Geometry &position,
    const internal_forces::ligaments::Characteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers) :
    m_compound(std::make_shared<internal_forces::Compound> (name, pathModifiers)),
    m_position(std::make_shared<internal_forces::Geometry>(position)),
    m_type(std::make_shared<internal_forces::ligaments::LIGAMENT_TYPE>(internal_forces::ligaments::LIGAMENT_TYPE::NO_LIGAMENT_TYPE)),
    m_characteristics(std::make_shared<internal_forces::ligaments::Characteristics>
                      (characteristics))
{

}

internal_forces::ligaments::Ligament::Ligament(const internal_forces::ligaments::Ligament &other) :
    m_compound(other.m_compound),
    m_position(other.m_position),
    m_type(other.m_type),
    m_characteristics(other.m_characteristics)
{

}

internal_forces::ligaments::Ligament::Ligament(
        const std::shared_ptr<internal_forces::ligaments::Ligament> other) :
    m_compound(other->m_compound),
    m_position(other->m_position),
    m_type(other->m_type),
    m_characteristics(other->m_characteristics)
{

}

internal_forces::ligaments::Ligament::~Ligament()
{
    //dtor
}

void internal_forces::ligaments::Ligament::DeepCopy(const internal_forces::ligaments::Ligament &other)
{
    *m_compound = *other.m_compound;
    *m_position = other.m_position->DeepCopy();
    *m_type = *other.m_type;
    *m_characteristics = other.m_characteristics->DeepCopy();
}

internal_forces::ligaments::LIGAMENT_TYPE internal_forces::ligaments::Ligament::type() const
{
    return *m_type;
}

void internal_forces::ligaments::Ligament::setType()
{
    *m_type = internal_forces::ligaments::LIGAMENT_TYPE::NO_LIGAMENT_TYPE;
}

void internal_forces::ligaments::Ligament::updateOrientations(
    rigidbody::Joints& model,
    const rigidbody::GeneralizedCoordinates &Q,
    int updateKin)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(model,*m_pathChanger,&Q,nullptr,updateKin);
}
void internal_forces::ligaments::Ligament::updateOrientations(
    rigidbody::Joints& model,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    int updateKin)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(model,*m_pathChanger,&Q,&Qdot, updateKin);
}
void internal_forces::ligaments::Ligament::updateOrientations(
    std::vector<utils::Vector3d>& ligamentPointsInGlobal,
    utils::Matrix &jacoPointsInGlobal)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(ligamentPointsInGlobal,jacoPointsInGlobal,nullptr);
}
void internal_forces::ligaments::Ligament::updateOrientations(
    std::vector<utils::Vector3d>& ligamentPointsInGlobal,
    utils::Matrix &jacoPointsInGlobal,
    const rigidbody::GeneralizedVelocity &Qdot)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(ligamentPointsInGlobal,jacoPointsInGlobal,&Qdot);
}

void internal_forces::ligaments::Ligament::setPosition(
    const internal_forces::Geometry &positions)
{
    *m_position = positions;
}
const internal_forces::Geometry &internal_forces::ligaments::Ligament::position() const
{
    return *m_position;
}

const utils::Scalar& internal_forces::ligaments::Ligament::length(
    rigidbody::Joints& model,
    const rigidbody::GeneralizedCoordinates &Q,
    int updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = 2;
#endif
    if (updateKin != 0) {
        m_position->updateKinematics(model,*m_pathChanger,&Q,nullptr,updateKin);
    }

    return position().length();
}

const utils::Scalar& internal_forces::ligaments::Ligament::velocity(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = true;
#endif
    if (updateKin) {
        m_position->updateKinematics(model,*m_pathChanger,&Q,&Qdot);
    }

    return m_position->velocity();
}

const utils::Scalar& internal_forces::ligaments::Ligament::force(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    int updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = 2;
#endif
    // Update the configuration
    if (updateKin == 1) {
        updateOrientations(model,Q,Qdot,1);
    } else if (updateKin == 2) {
        updateOrientations(model,Q,Qdot,2);
    } else {
        utils::Error::check(updateKin == 0,
                                    "Wrong level of update in force function");
    }

    // Computation
    computeFl();
    computeForce();
    return *m_force;
}

const utils::Scalar& internal_forces::ligaments::Ligament::force(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q,
    int updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = 2;
#endif
    // Update the configuration
    if (updateKin == 1) {
        updateOrientations(model,Q,1);
    } else if (updateKin == 2) {
        updateOrientations(model,Q,2);
    } else {
        utils::Error::check(updateKin == 0,
                                    "Wrong level of update in force function");
    }

    // Computation
    computeFl();
    computeForce();
    return *m_force;
}

void internal_forces::ligaments::Ligament::computeForce()
{
    *m_force = getForce();
}

utils::Scalar internal_forces::ligaments::Ligament::getForce()
{
    utils::Scalar damping_param;
    return *m_Fl + damping();
}

const std::vector<utils::Vector3d>& internal_forces::ligaments::Ligament::ligamentsPointsInGlobal(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q)
{
    m_position->updateKinematics(model,*m_pathChanger,&Q);

    return ligamentsPointsInGlobal();
}

const std::vector<utils::Vector3d>&
internal_forces::ligaments::Ligament::ligamentsPointsInGlobal() const
{
    return m_position->pointsInGlobal();
}

void internal_forces::ligaments::Ligament::setCharacteristics(
    const internal_forces::ligaments::Characteristics &characteristics)
{
    *m_characteristics = characteristics;
}

const internal_forces::ligaments::Characteristics&
internal_forces::ligaments::Ligament::characteristics() const
{
    return *m_characteristics;
}

const utils::Scalar& internal_forces::ligaments::Ligament::Fl()
{
    computeFl();
    return *m_Fl;
}

const utils::Scalar& internal_forces::ligaments::Ligament::damping()
{
    computeDamping();
    return *m_damping;
}

void internal_forces::ligaments::Ligament::computeDamping()
{


#ifdef BIORBD_USE_CASADI_MATH
    *m_damping = IF_ELSE_NAMESPACE::if_else_zero(
                  IF_ELSE_NAMESPACE::gt(position().velocity(), 0),
                ((position().velocity() / m_characteristics->maxShorteningSpeed())
                * m_characteristics->dampingParam()));

#else
    if (position().velocity() > 0) {
        *m_damping = (position().velocity()/ m_characteristics->maxShorteningSpeed()) * m_characteristics->dampingParam();
    } else {
        *m_damping = 0;
    }
#endif

}
