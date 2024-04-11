#define BIORBD_API_EXPORTS

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "Utils/RotoTrans.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "InternalForces/WrappingObject.h"
#include "InternalForces/PathModifiers.h"
#include "InternalForces/ViaPoint.h"
#include "InternalForces/Geometry.h"
#include "InternalForces/Muscles/Characteristics.h"
#include "InternalForces/Muscles/MuscleGeometry.h"

using namespace BIORBD_NAMESPACE;

internal_forces::muscles::MuscleGeometry::MuscleGeometry() :
    internal_forces::Geometry(),
    m_muscleTendonLength(std::make_shared<utils::Scalar>(0)),
    m_muscleLength(std::make_shared<utils::Scalar>(0))

{

}

internal_forces::muscles::MuscleGeometry::MuscleGeometry(
    const utils::Vector3d &origin,
    const utils::Vector3d &insertion) :
    internal_forces::Geometry(origin, insertion),
    m_muscleTendonLength(std::make_shared<utils::Scalar>(0)),
    m_muscleLength(std::make_shared<utils::Scalar>(0))
{

}

internal_forces::muscles::MuscleGeometry internal_forces::muscles::MuscleGeometry::DeepCopy() const
{
    internal_forces::muscles::MuscleGeometry copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::muscles::MuscleGeometry::DeepCopy(const internal_forces::muscles::MuscleGeometry &other)
{
    internal_forces::Geometry::DeepCopy(other);
    *m_muscleLength = *other.m_muscleLength;
    *m_muscleTendonLength = *other.m_muscleTendonLength;
}


// ------ PUBLIC FUNCTIONS ------ //
void internal_forces::muscles::MuscleGeometry::updateKinematics(
    rigidbody::Joints& updatedModel,
    const internal_forces::muscles::Characteristics& characteristics,
    const rigidbody::GeneralizedCoordinates *Q,
    const rigidbody::GeneralizedVelocity *Qdot)
{
    *m_posAndJacoWereForced = false;

    // Position of the points in space
    setPointsInGlobal(updatedModel, *Q);

    // Compute the Jacobian of the muscle points
    jacobian(updatedModel, *Q);

    // Complete the update
    _updateKinematics(Qdot, &characteristics);
}

void internal_forces::muscles::MuscleGeometry::updateKinematics(
    rigidbody::Joints& updatedModel,
    const internal_forces::muscles::Characteristics& characteristics,
    internal_forces::PathModifiers &pathModifiers,
    const rigidbody::GeneralizedCoordinates *Q,
    const rigidbody::GeneralizedVelocity *Qdot)
{
    *m_posAndJacoWereForced = false;

    // Position of the points in space
    setPointsInGlobal(updatedModel, *Q, &pathModifiers);

    // Compute the Jacobian of the muscle points
    jacobian(updatedModel, *Q);

    // Complete the update
    _updateKinematics(Qdot, &characteristics, &pathModifiers);
}

void internal_forces::muscles::MuscleGeometry::updateKinematics(
    std::vector<utils::Vector3d>& pointsInGlobal,
    utils::Matrix& jacoPointsInGlobal,
    const internal_forces::muscles::Characteristics& characteristics,
    const rigidbody::GeneralizedVelocity* Qdot)
{
    *m_posAndJacoWereForced = true;

    // Position of the points in space
    setPointsInGlobal(pointsInGlobal);

    // Compute the Jacobian of the muscle points
    jacobian(jacoPointsInGlobal);

    // Complete the update
    _updateKinematics(Qdot, &characteristics);
}

const utils::Scalar& internal_forces::muscles::MuscleGeometry::length() const
{
    utils::Error::check(
        *m_isGeometryComputed, "Geometry must be computed at least before calling length()");
    return *m_muscleLength;
}

const utils::Scalar& internal_forces::muscles::MuscleGeometry::musculoTendonLength() const
{
    utils::Error::check(
        *m_isGeometryComputed, "Geometry must be computed at least before calling length()");
    return *m_muscleTendonLength;
}

// --------------------------------------- //

void internal_forces::muscles::MuscleGeometry::_updateKinematics(
    const rigidbody::GeneralizedVelocity* Qdot,
    const internal_forces::muscles::Characteristics* characteristics,
    internal_forces::PathModifiers *pathModifiers)
{
    // Compute the length and velocities
    length(characteristics, pathModifiers);
    *m_isGeometryComputed = true;

    // Compute the jacobian of the lengths
    computeJacobianLength();
    if (Qdot != nullptr) {
        velocity(*Qdot);
        *m_isVelocityComputed = true;
    } else {
        *m_isVelocityComputed = false;
    }
}

const utils::Scalar& internal_forces::muscles::MuscleGeometry::length(
    const internal_forces::muscles::Characteristics* characteristics,
    internal_forces::PathModifiers *pathModifiers)
{
    *m_muscleTendonLength = 0;

    // because we can't combine, test the first (0) will let us know all the types if more than one
    if (pathModifiers != nullptr && pathModifiers->nbWraps()!=0) {
        utils::Error::check(
            pathModifiers->nbVia() == 0, "Cannot mix wrapping and via points yet" ) ;
        utils::Error::check(
            pathModifiers->nbWraps() < 2, "Cannot compute more than one wrapping yet");

        utils::Vector3d pi_wrap(0, 0, 0); // point on the wrapping related to insertion
        utils::Vector3d po_wrap(0, 0, 0); // point on the wrapping related to origin
        utils::Scalar lengthWrap(0);
        static_cast<internal_forces::WrappingObject&>(
            pathModifiers->object(0)).wrapPoints(po_wrap, pi_wrap, &lengthWrap);
        
        *m_muscleTendonLength = 
            ((*m_pointsInGlobal)[0] - po_wrap).norm()   + // length before the wrap
            lengthWrap                                  + // length on the wrap
            ((*m_pointsInGlobal)[3] - pi_wrap).norm();    // length after the wrap

    } else {
        for (size_t i=0; i<m_pointsInGlobal->size()-1; ++i) {
            *m_muscleTendonLength += ((*m_pointsInGlobal)[i+1] - (*m_pointsInGlobal)[i]).norm();
        }
    }
    *m_muscleLength = (*m_muscleTendonLength - characteristics->tendonSlackLength())/std::cos(characteristics->pennationAngle());
    return *m_muscleLength;
}
