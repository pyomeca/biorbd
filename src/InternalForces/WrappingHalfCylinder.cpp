#define BIORBD_API_EXPORTS
#include "InternalForces/WrappingHalfCylinder.h"

#include "Utils/String.h"
#include "Utils/RotoTrans.h"
#include "Utils/Vector2d.h"
#include "Utils/Matrix2d.h"
#include "RigidBody/Joints.h"

#ifdef USE_SMOOTH_IF_ELSE
#include "Utils/CasadiExpand.h"
#endif

using namespace BIORBD_NAMESPACE;

internal_forces::WrappingHalfCylinder::WrappingHalfCylinder() :
    internal_forces::WrappingObject (),
    m_radius(std::make_shared<utils::Scalar>(0)),
    m_length(std::make_shared<utils::Scalar>(0)),
    m_RTtoParent(std::make_shared<utils::RotoTrans>()),
    m_p1Wrap(std::make_shared<utils::Vector3d>()),
    m_p2Wrap(std::make_shared<utils::Vector3d>()),
    m_lengthAroundWrap(std::make_shared<utils::Scalar>(0))
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_HALF_CYLINDER;
}

internal_forces::WrappingHalfCylinder::WrappingHalfCylinder(
    const utils::Vector3d &other):
    internal_forces::WrappingObject(other)
{
    internal_forces::WrappingHalfCylinder& otherWrap(
        const_cast<internal_forces::WrappingHalfCylinder&>(
            dynamic_cast<const internal_forces::WrappingHalfCylinder&>(other)));
    m_radius = otherWrap.m_radius;
    m_length = otherWrap.m_length;
    m_RTtoParent = otherWrap.m_RTtoParent;
    m_p1Wrap = otherWrap.m_p1Wrap;
    m_p2Wrap = otherWrap.m_p2Wrap;
    m_lengthAroundWrap = otherWrap.m_lengthAroundWrap;
}

internal_forces::WrappingHalfCylinder::WrappingHalfCylinder(
    const utils::Vector3d *other):
    internal_forces::WrappingObject(*other)
{
    internal_forces::WrappingHalfCylinder* otherWrap(
        const_cast<internal_forces::WrappingHalfCylinder*>(
            dynamic_cast<const internal_forces::WrappingHalfCylinder*>(other)));
    m_radius = otherWrap->m_radius;
    m_length = otherWrap->m_length;
    m_RTtoParent = otherWrap->m_RTtoParent;
    m_p1Wrap = otherWrap->m_p1Wrap;
    m_p2Wrap = otherWrap->m_p2Wrap;
    m_lengthAroundWrap = otherWrap->m_lengthAroundWrap;
}

internal_forces::WrappingHalfCylinder::WrappingHalfCylinder(
    const utils::RotoTrans &rt,
    const utils::Scalar& radius,
    const utils::Scalar& length) :
    internal_forces::WrappingObject (rt.trans()),
    m_radius(std::make_shared<utils::Scalar>(radius)),
    m_length(std::make_shared<utils::Scalar>(length)),
    m_RTtoParent(std::make_shared<utils::RotoTrans>(rt)),
    m_p1Wrap(std::make_shared<utils::Vector3d>()),
    m_p2Wrap(std::make_shared<utils::Vector3d>()),
    m_lengthAroundWrap(std::make_shared<utils::Scalar>(0))
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_HALF_CYLINDER;
}

internal_forces::WrappingHalfCylinder::WrappingHalfCylinder(
    const utils::RotoTrans &rt,
    const utils::Scalar& radius,
    const utils::Scalar& length,
    const utils::String &name,
    const utils::String &parentName) :
    internal_forces::WrappingObject (rt.trans(), name, parentName),
    m_radius(std::make_shared<utils::Scalar>(radius)),
    m_length(std::make_shared<utils::Scalar>(length)),
    m_RTtoParent(std::make_shared<utils::RotoTrans>(rt)),
    m_p1Wrap(std::make_shared<utils::Vector3d>()),
    m_p2Wrap(std::make_shared<utils::Vector3d>()),
    m_lengthAroundWrap(std::make_shared<utils::Scalar>(0))
{
    *m_typeOfNode = utils::NODE_TYPE::WRAPPING_HALF_CYLINDER;
}

internal_forces::WrappingHalfCylinder
internal_forces::WrappingHalfCylinder::DeepCopy() const
{
    internal_forces::WrappingHalfCylinder copy;
    copy.DeepCopy(*this);
    return copy;
}

void internal_forces::WrappingHalfCylinder::DeepCopy(const
        internal_forces::WrappingHalfCylinder &other)
{
    internal_forces::WrappingObject::DeepCopy(other);
    *m_radius = *other.m_radius;
    *m_length = *other.m_length;
    *m_RTtoParent = *other.m_RTtoParent;
    *m_p1Wrap = other.m_p1Wrap->DeepCopy();
    *m_p2Wrap = other.m_p2Wrap->DeepCopy();
    *m_lengthAroundWrap = *other.m_lengthAroundWrap;
}

void internal_forces::WrappingHalfCylinder::wrapPoints(
    const utils::RotoTrans& rt,
    const utils::Vector3d& p1_bone,
    const utils::Vector3d& p2_bone,
    utils::Vector3d& p1,
    utils::Vector3d& p2,
    utils::Scalar *length)
{
    // This function takes the position of the wrapping and finds the location where muscle 1 and 2 leave the wrapping object

    // Find the nodes in the RT reference (of the cylinder)
    NodeMusclePair p_glob(p1_bone, p2_bone);
    p_glob.m_p1->applyRT(rt.transpose());
    p_glob.m_p2->applyRT(rt.transpose());

    // Find the tangents of these points to the circle (cylinder seen from above)
    utils::Vector3d p1_tan(0, 0, 0);
    utils::Vector3d p2_tan(0, 0, 0);
    findTangentToCircle(*p_glob.m_p1, p1_tan);
    findTangentToCircle(*p_glob.m_p2, p2_tan);


    // Find the vertical component
    NodeMusclePair tanPoints(p1_tan, p2_tan);

    // if the wrap is not supposed to happen 
    // if there is a straight line in between two points not passing throught the cylinder
    if(!findVerticalNode(p_glob, tanPoints)){ 
        // add the two wrapping points on that streight line
        // each one at one third of length
        Vector3d vec((*p_glob.m_p2 - *p_glob.m_p1)/3);
        p1_tan = *p_glob.m_p1 + vec;
        p2_tan = p1_tan + vec;
        tanPoints = NodeMusclePair(p1_tan, p2_tan);
    }

    // If asked, compute the distance distance traveled on the periphery of the cylinder
    // Apply pythagorus to the cercle arc
    if (length != nullptr) { // If it is not nullptr
        *length = computeLength(tanPoints);
    }

    // Reset the points in global (space)
    tanPoints.m_p1->applyRT(rt);
    tanPoints.m_p2->applyRT(rt);

    // Reset the desired values
    p1 = *tanPoints.m_p1;
    p2 = *tanPoints.m_p2;

    // Store the values for a futur call
    m_p1Wrap = tanPoints.m_p1;
    m_p2Wrap = tanPoints.m_p2;
    if (length != nullptr) { // If it is not nullptr
        *m_lengthAroundWrap = *length;
    }
}

void internal_forces::WrappingHalfCylinder::wrapPoints(
    rigidbody::Joints& model,
    const rigidbody::GeneralizedCoordinates& Q,
    const utils::Vector3d& p1_bone,
    const utils::Vector3d& p2_bone,
    utils::Vector3d& p1,
    utils::Vector3d& p2,
    utils::Scalar *length)
{
    // This function takes a model and a position of the model and returns the location where muscle 1 et 2 leave the wrapping object

    wrapPoints(RT(model,Q), p1_bone, p2_bone, p1, p2, length);
}

void internal_forces::WrappingHalfCylinder::wrapPoints(
    utils::Vector3d& p1,
    utils::Vector3d& p2,
    utils::Scalar *length)
{
    p1 = *m_p1Wrap;
    p2 = *m_p2Wrap;
    if (length != nullptr) {
        *length = *m_lengthAroundWrap;
    }
}

const utils::RotoTrans& internal_forces::WrappingHalfCylinder::RT(
    rigidbody::Joints &model,
    const rigidbody::GeneralizedCoordinates& Q,
    bool updateKin)
{
    // Get the RotoTrans matrix of the cylinder in space
    *m_RT = model.globalJCS(Q, *m_parentName, updateKin) * *m_RTtoParent;
    return *m_RT;
}

void internal_forces::WrappingHalfCylinder::setRadius(
    const utils::Scalar &val)
{
    *m_radius = val;
}

utils::Scalar internal_forces::WrappingHalfCylinder::radius() const
{
    return *m_radius;
}

utils::Scalar internal_forces::WrappingHalfCylinder::diameter() const
{
    return 2 * *m_radius;
}

void internal_forces::WrappingHalfCylinder::setLength(
    const utils::Scalar& val)
{
    *m_length = val;
}

const utils::Scalar& internal_forces::WrappingHalfCylinder::length()
const
{
    return *m_length;
}

void internal_forces::WrappingHalfCylinder::findTangentToCircle(
    const utils::Vector3d& p,
    utils::Vector3d& p_tan) const
{
    // This function ignores the Z axis of the vector p to create the circle
#ifdef BIORBD_USE_EIGEN3_MATH
    utils::Scalar p_dot = static_cast<Eigen::Vector2d>(p.block(0,0,2,1)).dot(static_cast<Eigen::Vector2d>(p.block(0,0,2,1)));
#else
    utils::Scalar p_dot = p.block(0,0,2,1).dot(p.block(0,0,2,1));
#endif

    const utils::Vector2d& Q0(radius()*radius()/p_dot*p.block(0,0,
            2,1));
    utils::Matrix2d tp(utils::Matrix2d::Zero());
    tp(0, 1) = -1;
    tp(1, 0) = 1;

    const utils::Vector2d& T(
        radius()/p_dot*std::sqrt(p_dot-radius()*radius()) * tp * p.block(0,0,2,1));

    // GEt the tangent on both sides
    NodeMusclePair m(p,p);
    m.m_p1->block(0,0,2,1) = Q0 + T;
    m.m_p2->block(0,0,2,1) = Q0 - T;

    // Select on of the two tangents
    selectTangents(m, p_tan);
}

void internal_forces::WrappingHalfCylinder::selectTangents(
    const NodeMusclePair &p1,
    utils::Vector3d &p_tan) const
{
#ifdef BIORBD_USE_CASADI_MATH
    p_tan = IF_ELSE_NAMESPACE::if_else(
                IF_ELSE_NAMESPACE::ge((*p1.m_p2)(0), (*p1.m_p1)(0)),
                *p1.m_p2, *p1.m_p1);
#else
    if ((*p1.m_p2)(0) >= (*p1.m_p1)(0)) {
        p_tan = *p1.m_p2;
    } else {
        p_tan = *p1.m_p1;
    }
#endif

}
bool internal_forces::WrappingHalfCylinder::findVerticalNode(
    const NodeMusclePair &pointsInGlobal,
    NodeMusclePair &pointsToWrap) const
{
    // Before everything, make sure the point wrap
#ifdef BIORBD_USE_CASADI_MATH
    // In CASADI, we have to assume it does...
#else
    if (!checkIfWraps(pointsInGlobal,
                      pointsToWrap)) { // If it doesn't pass by the wrap, put NaN and stop
        for (size_t i=0; i<3; ++i) {
            (*pointsToWrap.m_p1)(i) = static_cast<utils::Scalar>
                                      (static_cast<double>(NAN));
            (*pointsToWrap.m_p2)(i) = static_cast<utils::Scalar>
                                      (static_cast<double>(NAN));
        }
        return false;
    } else {
        // Make sure the z component won't cause any problem in the rotation computation
        (*pointsToWrap.m_p1)(2) = 0;
        (*pointsToWrap.m_p2)(2) = 0;
    }
#endif

    // Strategy : Find the matrix of the passage between the aligned points in x and the cylinder.
    // Find the location where the points cross the cylinder

    // X is the straight line between the two points
    utils::Vector3d X(*pointsInGlobal.m_p2 - *pointsInGlobal.m_p1);
    // Z is the empty axis of the cylinder
    utils::Vector3d Z(0,0,1);

    utils::Vector3d Y(Z.cross(X));
    // Re-compute X for it to be aligned with the cylinder
    X = Y.cross(Z);
    // Normalise everything
    X = X/X.norm();
    Y = Y/Y.norm();
    Z = Z/Z.norm();
    // Concatenate to get the rotation matrix
    utils::RotoTrans R(X(0), X(1), X(2), 0,
                               Y(0), Y(1), Y(2), 0,
                               Z(0), Z(1), Z(2), 0,
                               0,    0,    0,    1);

    // Turn the points in the R reference
    utils::Vector3d globA(*pointsInGlobal.m_p1);
    utils::Vector3d globB(*pointsInGlobal.m_p2);
    utils::Vector3d wrapA(*pointsToWrap.m_p1);
    utils::Vector3d wrapB(*pointsToWrap.m_p2);
    globA.applyRT(R);
    globB.applyRT(R);
    wrapA.applyRT(R);
    wrapB.applyRT(R);

    // The height depends on the relative distance
    (*pointsToWrap.m_p1)(2) = utils::Scalar(wrapA(0)-globB(0)) /
                              utils::Scalar(globA(0)-globB(0)) *
                              ((*pointsInGlobal.m_p1)(2)-(*pointsInGlobal.m_p2)(2)) + (*pointsInGlobal.m_p2)(
                                  2);
    (*pointsToWrap.m_p2)(2) = utils::Scalar(wrapB(0)-globB(0)) /
                              utils::Scalar(globA(0)-globB(0)) *
                              ((*pointsInGlobal.m_p1)(2)-(*pointsInGlobal.m_p2)(2)) + (*pointsInGlobal.m_p2)(
                                  2);

    return true;
}

#ifndef BIORBD_USE_CASADI_MATH
bool internal_forces::WrappingHalfCylinder::checkIfWraps(
    const NodeMusclePair &pointsInGlobal,
    NodeMusclePair &pointsToWrap) const
{
    // It seems that all this function is ignored up to the last check... Once
    // it is checked, validate the Casadi implementation

    // First quick tests
    // if both points are on the left and we have to go left
    if ((*pointsInGlobal.m_p1)(0) > radius()
            && (*pointsInGlobal.m_p2)(0) > radius()) {
        return false;
    }

    // If we are on top of the wrap, it is impossible to determine because the wrap
    // is not a cylinder but a half-cylinder
    if ( ( (*pointsInGlobal.m_p1)(1) > 0 && (*pointsInGlobal.m_p2)(1) > 0)
            || ( (*pointsInGlobal.m_p1)(1) < 0
                 && (*pointsInGlobal.m_p2)(1) < 0) ) {
        return false;
    }

    // If we have a height* smaller than the radius, there is a numerical aberation
    if ( fabs( (*pointsInGlobal.m_p1)(1)) < radius()
            || fabs( (*pointsInGlobal.m_p2)(1)) < radius() ) {
        return false;
    }

    // If we have reached this stage, one test is left
    // If the straight line between the two points go through the cylinder,there is a wrap
    if (    ( (*pointsToWrap.m_p1)(0) < (*pointsToWrap.m_p2)(0)
              && (*pointsInGlobal.m_p1)(0) > (*pointsInGlobal.m_p2)(0)) ||
            ( (*pointsToWrap.m_p1)(0) > (*pointsToWrap.m_p2)(0)
              && (*pointsInGlobal.m_p1)(0) < (*pointsInGlobal.m_p2)(0))   ) {
        return false;
    }

    // Return the answer
    return true;
}
#endif

utils::Scalar internal_forces::WrappingHalfCylinder::computeLength(
    const NodeMusclePair &p) const
{
    utils::Scalar arc = std::acos(    ( (*p.m_p1)(0) * (*p.m_p2)(0) +
                                (*p.m_p1)(1) * (*p.m_p2)(1))
                                /
                                std::sqrt( ((*p.m_p1)(0) * (*p.m_p1)(0) + (*p.m_p1)(1) * (*p.m_p1)(1)) *
                                           ( (*p.m_p2)(0) * (*p.m_p2)(0) + (*p.m_p2)(1) * (*p.m_p2)(1))   )
                                         ) * radius();

    return std::sqrt(arc*arc + ( (*p.m_p1)(2) - (*p.m_p2)(2)) * ( (*p.m_p1)(2) -
                     (*p.m_p2)(2))  );
}
