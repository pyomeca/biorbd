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

using namespace BIORBD_NAMESPACE;

internal_forces::Geometry::Geometry() :
    m_origin(std::make_shared<utils::Vector3d>()),
    m_insertion(std::make_shared<utils::Vector3d>()),
    m_originInGlobal(std::make_shared<utils::Vector3d>
                     (utils::Vector3d::Zero())),
    m_insertionInGlobal(std::make_shared<utils::Vector3d>
                        (utils::Vector3d::Zero())),
    m_pointsInGlobal(std::make_shared<std::vector<utils::Vector3d>>()),
    m_pointsInLocal(std::make_shared<std::vector<utils::Vector3d>>()),
    m_jacobian(std::make_shared<utils::Matrix>()),
    m_jacobianLength(std::make_shared<utils::Matrix>()),
    m_length(std::make_shared<utils::Scalar>(0)),
    m_velocity(std::make_shared<utils::Scalar>(0)),
    m_isGeometryComputed(std::make_shared<bool>(false)),
    m_isVelocityComputed(std::make_shared<bool>(false)),
    m_posAndJacoWereForced(std::make_shared<bool>(false))
{

}

internal_forces::Geometry::Geometry(
    const utils::Vector3d &origin,
    const utils::Vector3d &insertion) :
    m_origin(std::make_shared<utils::Vector3d>(origin)),
    m_insertion(std::make_shared<utils::Vector3d>(insertion)),
    m_originInGlobal(std::make_shared<utils::Vector3d>
                     (utils::Vector3d::Zero())),
    m_insertionInGlobal(std::make_shared<utils::Vector3d>
                        (utils::Vector3d::Zero())),
    m_pointsInGlobal(std::make_shared<std::vector<utils::Vector3d>>()),
    m_pointsInLocal(std::make_shared<std::vector<utils::Vector3d>>()),
    m_jacobian(std::make_shared<utils::Matrix>()),
    m_jacobianLength(std::make_shared<utils::Matrix>()),
    m_length(std::make_shared<utils::Scalar>(0)),
    m_velocity(std::make_shared<utils::Scalar>(0)),
    m_isGeometryComputed(std::make_shared<bool>(false)),
    m_isVelocityComputed(std::make_shared<bool>(false)),
    m_posAndJacoWereForced(std::make_shared<bool>(false))
{

}

internal_forces::Geometry internal_forces::Geometry::DeepCopy() const
{
    internal_forces::Geometry copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::Geometry::DeepCopy(const internal_forces::Geometry &other)
{
    *m_origin = other.m_origin->DeepCopy();
    *m_insertion = other.m_insertion->DeepCopy();
    *m_originInGlobal = other.m_originInGlobal->DeepCopy();
    *m_insertionInGlobal = other.m_insertionInGlobal->DeepCopy();
    m_pointsInGlobal->resize(other.m_pointsInGlobal->size());
    for (size_t i=0; i<other.m_pointsInGlobal->size(); ++i) {
        (*m_pointsInGlobal)[i] = (*other.m_pointsInGlobal)[i].DeepCopy();
    }
    m_pointsInLocal->resize(other.m_pointsInLocal->size());
    for (size_t i=0; i<other.m_pointsInLocal->size(); ++i) {
        (*m_pointsInLocal)[i] = (*other.m_pointsInLocal)[i].DeepCopy();
    }
    *m_jacobian = *other.m_jacobian;
    *m_jacobianLength = *other.m_jacobianLength;
    *m_length = *other.m_length;
    *m_velocity = *other.m_velocity;
    *m_isGeometryComputed = *other.m_isGeometryComputed;
    *m_isVelocityComputed = *other.m_isVelocityComputed;
    *m_posAndJacoWereForced = *other.m_posAndJacoWereForced;
}


// ------ PUBLIC FUNCTIONS ------ //
void internal_forces::Geometry::updateKinematics(
    rigidbody::Joints &updatedModel,
    const rigidbody::GeneralizedCoordinates *Q,
    const rigidbody::GeneralizedVelocity *Qdot)
{
    *m_posAndJacoWereForced = false;

    // Position of the points in space
    setPointsInGlobal(updatedModel, *Q);

    // Compute the Jacobian of the muscle points
    jacobian(updatedModel, *Q);

    // Complete the update
    _updateKinematics(Qdot, nullptr);
}

void internal_forces::Geometry::updateKinematics(
    rigidbody::Joints& updatedModel,
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
    _updateKinematics(Qdot, &pathModifiers);
}

void internal_forces::Geometry::updateKinematics(
    std::vector<utils::Vector3d>& pointsInGlobal,
    utils::Matrix& jacoPointsInGlobal,
    const rigidbody::GeneralizedVelocity* Qdot)
{
    *m_posAndJacoWereForced = true;

    // Position of the points in space
    setPointsInGlobal(pointsInGlobal);

    // Compute the Jacobian of the muscle points
    jacobian(jacoPointsInGlobal);

    // Complete the update
    _updateKinematics(Qdot, nullptr);
}

// Get and set the positions of the origins and insertions
void internal_forces::Geometry::setOrigin(
    const utils::Vector3d &position)
{
    if (dynamic_cast<const rigidbody::NodeSegment*>(&position)) {
        *m_origin = position;
    } else {
        // Preserve the Node information
        m_origin->RigidBodyDynamics::Math::Vector3d::operator=(position);
    }
}
const utils::Vector3d& internal_forces::Geometry::originInLocal() const
{
    return *m_origin;
}

void internal_forces::Geometry::setInsertionInLocal(
    const utils::Vector3d &position)
{
    if (dynamic_cast<const rigidbody::NodeSegment*>(&position)) {
        *m_insertion = position;
    } else {
        // Preserve the Node information
        m_insertion->RigidBodyDynamics::Math::Vector3d::operator=(position);
    }
}
const utils::Vector3d &internal_forces::Geometry::insertionInLocal()
const
{
    return *m_insertion;
}

// Position of the muscles in space
const utils::Vector3d &internal_forces::Geometry::originInGlobal() const
{
    utils::Error::check(
        *m_isGeometryComputed, "Geometry must be computed at least once before calling originInLocal()");
    return *m_originInGlobal;
}
const utils::Vector3d &internal_forces::Geometry::insertionInGlobal()
const
{
    utils::Error::check(
        *m_isGeometryComputed, "Geometry must be computed at least once before calling insertionInGlobal()");
    return *m_insertionInGlobal;
}
const std::vector<utils::Vector3d>
&internal_forces::Geometry::pointsInGlobal() const
{
    utils::Error::check(
        *m_isGeometryComputed, "Geometry must be computed at least once before calling musclesPointsInGlobal()");
    return *m_pointsInGlobal;
}

// Return the length and muscular velocity
const utils::Scalar& internal_forces::Geometry::length() const
{
    utils::Error::check(
        *m_isGeometryComputed, "Geometry must be computed at least before calling length()");
    return *m_length;
}

const utils::Scalar& internal_forces::Geometry::velocity() const
{
    utils::Error::check(
        *m_isVelocityComputed, "Geometry must be computed before calling velocity()");
    return *m_velocity;
}

// Return the Jacobian
const utils::Matrix& internal_forces::Geometry::jacobian() const
{
    utils::Error::check(
        *m_isGeometryComputed, "Geometry must be computed before calling jacobian()");
    return *m_jacobian;
} // Return the last Jacobian
utils::Matrix internal_forces::Geometry::jacobianOrigin() const
{
    utils::Error::check(
        *m_isGeometryComputed, "Geometry must be computed before calling jacobianOrigin()");
    return m_jacobian->block(0,0,3,m_jacobian->cols());
}
utils::Matrix internal_forces::Geometry::jacobianInsertion() const
{
    utils::Error::check(
        *m_isGeometryComputed, "Geometry must be computed before calling jacobianInsertion()");
    return m_jacobian->block(m_jacobian->rows()-3,0,3,m_jacobian->cols());
}
utils::Matrix internal_forces::Geometry::jacobian(
    size_t idxViaPoint) const
{
    utils::Error::check(
        *m_isGeometryComputed, "Geometry must be computed before calling jacobian(i)");
    return m_jacobian->block(3* static_cast<unsigned int>(idxViaPoint),0,3,m_jacobian->cols());
}

const utils::Matrix &internal_forces::Geometry::jacobianLength() const
{
    utils::Error::check(
        *m_isGeometryComputed, "Geometry must be computed before calling jacobianLength()");
    return *m_jacobianLength;
}

// --------------------------------------- //

void internal_forces::Geometry::_updateKinematics(
    const rigidbody::GeneralizedVelocity* Qdot,
    internal_forces::PathModifiers *pathModifiers)
{
    // Compute the length and velocities
    length(pathModifiers);
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

const utils::Vector3d &internal_forces::Geometry::originInGlobal(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q)
{
    // Return the position of the marker in function of the given position
    m_originInGlobal->block(0,0,3,1) = model.CalcBodyToBaseCoordinates(Q, m_origin->parent(), *m_origin, false);
    return *m_originInGlobal;
}

const utils::Vector3d &internal_forces::Geometry::insertionInGlobal(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q)
{

    // Return the position of the marker in function of the given position
    m_insertionInGlobal->block(0,0,3,1) = model.CalcBodyToBaseCoordinates(Q, m_insertion->parent(), *m_insertion, false);
    return *m_insertionInGlobal;
}

void internal_forces::Geometry::setPointsInGlobal(
    std::vector<utils::Vector3d> &ptsInGlobal)
{
    utils::Error::check(ptsInGlobal.size() >= 2,
                                "ptsInGlobal must at least have an origin and an insertion");
    m_pointsInLocal->clear(); // In this mode, we don't need the local, because the Jacobian of the points has to be given as well
    *m_pointsInGlobal = ptsInGlobal;
}

void internal_forces::Geometry::setPointsInGlobal(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q,
    internal_forces::PathModifiers *pathModifiers)
{
    // Output varible (reset to zero)
    m_pointsInLocal->clear();
    m_pointsInGlobal->clear();

    // Do not apply on wrapping objects
    if (pathModifiers->nbWraps()!=0) {
        // CHECK TO MODIFY BEFOR GOING FORWARD WITH PROJECTS
        utils::Error::check(
            pathModifiers->nbVia() == 0, "Cannot mix wrapping and via points yet") ;
        utils::Error::check(
            pathModifiers->nbWraps() < 2, "Cannot compute more than one wrapping yet");

        // Get the matrix of Rt of the wrap
        internal_forces::WrappingObject& w =
            static_cast<internal_forces::WrappingObject&>(pathModifiers->object(0));
        const utils::RotoTrans& RT = w.RT(model, Q, false);

        // Alias
        const utils::Vector3d& po_mus = originInGlobal(model, Q);  // Origin on bone
        const utils::Vector3d& pi_mus = insertionInGlobal(model, Q); // Insertion on bone

        utils::Vector3d pi_wrap(0, 0, 0); // point on the wrapping related to insertion
        utils::Vector3d po_wrap(0, 0, 0); // point on the wrapping related to origin

        utils::Scalar a; // Force the computation of the length
        w.wrapPoints(RT,po_mus,pi_mus,po_wrap, pi_wrap, &a);

        // Store the points in local
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInLocal->push_back(
            utils::Vector3d(
                model.CalcBodyToBaseCoordinates(Q, w.parent(), po_wrap, false), "wrap_o", w.parent()));
        m_pointsInLocal->push_back(
            utils::Vector3d(
                model.CalcBodyToBaseCoordinates(Q, w.parent(), pi_wrap, false), "wrap_i", w.parent()));
        m_pointsInLocal->push_back(insertionInLocal());

        // Store the points in global
        m_pointsInGlobal->push_back(po_mus);
        m_pointsInGlobal->push_back(po_wrap);
        m_pointsInGlobal->push_back(pi_wrap);
        m_pointsInGlobal->push_back(pi_mus);

    }

    else if (pathModifiers->nbObjects()!=0
             && pathModifiers->object(0).typeOfNode() == utils::NODE_TYPE::VIA_POINT) {
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInGlobal->push_back(originInGlobal(model, Q));
        for (size_t i=0; i<pathModifiers->nbObjects(); ++i) {
            const internal_forces::ViaPoint& node(static_cast<internal_forces::ViaPoint&>(pathModifiers->object(i)));
            m_pointsInLocal->push_back(node);
            m_pointsInGlobal->push_back(model.CalcBodyToBaseCoordinates(Q, node.parent(), node, false));
        }
        m_pointsInLocal->push_back(insertionInLocal());
        m_pointsInGlobal->push_back(insertionInGlobal(model,Q));

    } else if (pathModifiers->nbObjects()==0) {
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInLocal->push_back(insertionInLocal());
        m_pointsInGlobal->push_back(originInGlobal(model, Q));
        m_pointsInGlobal->push_back(insertionInGlobal(model,Q));
    } else {
        utils::Error::raise("Length for this type of object was not implemented");
    }

    // Set the dimension of jacobian
    setJacobianDimension(model);
}

const utils::Scalar& internal_forces::Geometry::length(
    internal_forces::PathModifiers *pathModifiers)
{
    *m_length = 0;

    // because we can't combine, test the first (0) will let us know all the types if more than one
    if (pathModifiers != nullptr && pathModifiers->nbWraps()!=0) {
        utils::Error::check(pathModifiers->nbVia() == 0,
                                    "Cannot mix wrapping and via points yet" ) ;
        utils::Error::check(pathModifiers->nbWraps() < 2,
                                    "Cannot compute more than one wrapping yet");

        utils::Vector3d pi_wrap(0, 0, 0); // point on the wrapping related to insertion
        utils::Vector3d po_wrap(0, 0, 0); // point on the wrapping related to origin
        utils::Scalar lengthWrap(0);
        static_cast<internal_forces::WrappingObject&>(
            pathModifiers->object(0)).wrapPoints(po_wrap, pi_wrap, &lengthWrap);
        *m_length = ((*m_pointsInGlobal)[0] - po_wrap).norm()
                                + // length before the wrap
                                lengthWrap                 + // length on the wrap
                                ((*m_pointsInGlobal)[3] - pi_wrap).norm();   // length after the wrap

    } else {
        for (size_t i=0; i<m_pointsInGlobal->size()-1; ++i) {
            *m_length += ((*m_pointsInGlobal)[i+1] - (*m_pointsInGlobal)[i]).norm();
        }
    }
    return *m_length;
}

const utils::Scalar& internal_forces::Geometry::velocity(
    const rigidbody::GeneralizedVelocity &Qdot)
{
    // Compute the velocity of the muscular elongation
    *m_velocity = (jacobianLength()*Qdot)[0];
    return *m_velocity;
}

void internal_forces::Geometry::setJacobianDimension(
    rigidbody::Joints& model)
{
    *m_jacobian = utils::Matrix::Zero(static_cast<unsigned int>(m_pointsInLocal->size()*3), model.dof_count);
}

void internal_forces::Geometry::jacobian(const utils::Matrix &jaco)
{
    utils::Error::check(jaco.rows()/3 == static_cast<int>(m_pointsInGlobal->size()), "Jacobian is the wrong size");
    *m_jacobian = jaco;
}

void internal_forces::Geometry::jacobian(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates &Q)
{
    // jacobian is a protected method, so all method that calls it is expected to have updated the kinematics if required
    bool updateKin = false;
    for (size_t i=0; i<m_pointsInLocal->size(); ++i) {
        m_jacobian->block(3* static_cast<unsigned int>(i),0,3,model.dof_count) =
            model.CalcPointJacobian(Q, (*m_pointsInLocal)[i].parent(), (*m_pointsInLocal)[i], updateKin);
    }
}

void internal_forces::Geometry::computeJacobianLength()
{
    *m_jacobianLength = utils::Matrix::Zero(1, m_jacobian->cols());

    // jacobian approximates as if there were no wrapping object
    const std::vector<utils::Vector3d>& p = *m_pointsInGlobal;
    for (size_t i=0; i<p.size()-1 ; ++i) {
        *m_jacobianLength += (( p[i+1] - p[i] ).transpose() * (jacobian(i+1) - jacobian(
                                  i)))
                             /
                             ( p[i+1] - p[i] ).norm();
    }
}
