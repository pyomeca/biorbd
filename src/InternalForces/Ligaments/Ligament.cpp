#define BIORBD_API_EXPORTS

#include "Utils/Error.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "InternalForces/PathModifiers.h"
#include "InternalForces/Compound.h"
#include "InternalForces/Geometry.h"
#include "InternalForces/Ligaments/LigamentsEnums.h"
#include "InternalForces/Ligaments/LigamentCharacteristics.h"
#include "InternalForces/Ligaments/Ligament.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;
internal_forces::ligaments::Ligament::Ligament() :
    internal_forces::Compound(),
    m_position(std::make_shared<internal_forces::Geometry>()),
    m_type(std::make_shared<internal_forces::ligaments::LIGAMENT_TYPE>(internal_forces::ligaments::LIGAMENT_TYPE::NO_LIGAMENT_TYPE)),
    m_characteristics(std::make_shared<internal_forces::ligaments::LigamentCharacteristics>()),
    m_Fl(std::make_shared<utils::Scalar>()),
    m_damping(std::make_shared<utils::Scalar>())

{
    setType();
}

internal_forces::ligaments::Ligament::Ligament(
    const utils::String & name,
    const internal_forces::Geometry & position,
    const internal_forces::ligaments::LigamentCharacteristics &characteristics) :
    internal_forces::Compound(name),
    m_position(std::make_shared<internal_forces::Geometry>(position)),
    m_type(std::make_shared<internal_forces::ligaments::LIGAMENT_TYPE>(internal_forces::ligaments::LIGAMENT_TYPE::NO_LIGAMENT_TYPE)),
    m_characteristics(std::make_shared<internal_forces::ligaments::LigamentCharacteristics>
                      (characteristics)),
    m_Fl(std::make_shared<utils::Scalar>()),
    m_damping(std::make_shared<utils::Scalar>())
{
    setType();
}

internal_forces::ligaments::Ligament::Ligament(
    const utils::String &name,
    const internal_forces::Geometry &position,
    const internal_forces::ligaments::LigamentCharacteristics &characteristics,
    const internal_forces::PathModifiers &pathModifiers) :
    internal_forces::Compound(name, pathModifiers),
    m_position(std::make_shared<internal_forces::Geometry>(position)),
    m_type(std::make_shared<internal_forces::ligaments::LIGAMENT_TYPE>(internal_forces::ligaments::LIGAMENT_TYPE::NO_LIGAMENT_TYPE)),
    m_characteristics(std::make_shared<internal_forces::ligaments::LigamentCharacteristics>
                      (characteristics)),
    m_Fl(std::make_shared<utils::Scalar>()),
    m_damping(std::make_shared<utils::Scalar>())
{
    setType();
}

internal_forces::ligaments::Ligament::Ligament(const internal_forces::ligaments::Ligament &other) :
    internal_forces::Compound (other),
    m_position(other.m_position),
    m_type(other.m_type),
    m_characteristics(other.m_characteristics),
    m_Fl(other.m_Fl),
    m_damping(other.m_damping)
{

}

internal_forces::ligaments::Ligament::Ligament(
        const std::shared_ptr<internal_forces::ligaments::Ligament> other) :
    internal_forces::Compound(other),
    m_position(other->m_position),
    m_type(other->m_type),
    m_characteristics(other->m_characteristics),
    m_Fl(other->m_Fl),
    m_damping(other->m_damping)
{

}

internal_forces::ligaments::Ligament::~Ligament()
{
    //dtor
}

void internal_forces::ligaments::Ligament::DeepCopy(const internal_forces::ligaments::Ligament &other)
{
    this->internal_forces::Compound::DeepCopy(other);
    *m_position = other.m_position->DeepCopy();
    *m_type = *other.m_type;
    *m_characteristics = other.m_characteristics->DeepCopy();
    *m_Fl = *other.m_Fl;
    *m_damping = *other.m_damping;
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
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates &Q)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(updatedModel, *m_pathChanger, &Q, nullptr);
}
void internal_forces::ligaments::Ligament::updateOrientations(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot)

{
    // Update de la position des insertions et origines
    m_position->updateKinematics(updatedModel, *m_pathChanger, &Q, &Qdot);
}
void internal_forces::ligaments::Ligament::updateOrientations(
    std::vector<utils::Vector3d>& ligamentPointsInGlobal,
    utils::Matrix &jacoPointsInGlobal)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(ligamentPointsInGlobal, jacoPointsInGlobal, nullptr);
}
void internal_forces::ligaments::Ligament::updateOrientations(
    std::vector<utils::Vector3d>& ligamentPointsInGlobal,
    utils::Matrix &jacoPointsInGlobal,
    const rigidbody::GeneralizedVelocity &Qdot)
{
    // Update de la position des insertions et origines
    m_position->updateKinematics(ligamentPointsInGlobal, jacoPointsInGlobal, &Qdot);
}

void internal_forces::ligaments::Ligament::setPosition(
    const internal_forces::Geometry &positions)
{
    *m_position = positions;
}
const internal_forces::Geometry& internal_forces::ligaments::Ligament::position() const
{
    return *m_position;
}

const utils::Scalar& internal_forces::ligaments::Ligament::length(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q,
    bool updateLigamentKinematics)
{
    if (updateLigamentKinematics) m_position->updateKinematics(updatedModel, *m_pathChanger, &Q, nullptr);
    return position().length();
}

const utils::Scalar& internal_forces::ligaments::Ligament::velocity(
    rigidbody::Joints& updatedModel,
    const rigidbody::GeneralizedCoordinates& Q,
    const rigidbody::GeneralizedVelocity& Qdot,
    bool updateLigamentKinematics)
{
    if (updateLigamentKinematics) {
        m_position->updateKinematics(updatedModel, *m_pathChanger, &Q, &Qdot);
    }

    return m_position->velocity();
}

const utils::Scalar& internal_forces::ligaments::Ligament::force(
    rigidbody::Joints &updatedModel,
    const rigidbody::GeneralizedCoordinates &Q,
    const rigidbody::GeneralizedVelocity &Qdot,
    bool updateLigamentKinematics)
{
    // Update the ligament configuration if necessary
    if (updateLigamentKinematics) updateOrientations(updatedModel, Q, Qdot);

    // Computation
    computeFl();
    computeForce();
    return *m_force;
}

const utils::Scalar& internal_forces::ligaments::Ligament::force(
    rigidbody::Joints &updatedModel,
    const rigidbody::GeneralizedCoordinates &Q,
    bool updateLigamentKinematics)
{
    // Update the ligament configuration if necessary
    if (updateLigamentKinematics) updateOrientations(updatedModel, Q);

    // Computation
    computeFl();
    computeForce();
    return *m_force;
}

const utils::Scalar& internal_forces::ligaments::Ligament::force()
{
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
    return *m_Fl + damping();
}

const std::vector<utils::Vector3d>& internal_forces::ligaments::Ligament::ligamentsPointsInGlobal(
    rigidbody::Joints &updatedModel,
    const rigidbody::GeneralizedCoordinates &Q, 
    bool updateLigamentKinematics)
{
    if (updateLigamentKinematics) m_position->updateKinematics(updatedModel, *m_pathChanger, &Q);
    return ligamentsPointsInGlobal();
}

const std::vector<utils::Vector3d>& internal_forces::ligaments::Ligament::ligamentsPointsInGlobal() const
{
    return m_position->pointsInGlobal();
}

void internal_forces::ligaments::Ligament::setCharacteristics(
    const internal_forces::ligaments::LigamentCharacteristics &characteristics)
{
    *m_characteristics = characteristics;
}

const internal_forces::ligaments::LigamentCharacteristics&
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
                ((position().velocity() / characteristics().maxShorteningSpeed())
                * characteristics().dampingParam()));

#else
    if (position().velocity() > 0) {
        *m_damping = (position().velocity()/ characteristics().maxShorteningSpeed()) * characteristics().dampingParam();
    } else {
        *m_damping = 0;
    }
#endif

}
