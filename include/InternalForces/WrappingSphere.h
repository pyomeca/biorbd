#ifndef BIORBD_MUSCLES_WRAPPING_SPHERE_H
#define BIORBD_MUSCLES_WRAPPING_SPHERE_H

#include "biorbdConfig.h"
#include "InternalForces/WrappingObject.h"

namespace BIORBD_NAMESPACE
{
namespace internal_forces
{
///
/// \brief Sphere object that makes the muscle to wrap around
///
class BIORBD_API WrappingSphere : public WrappingObject
{
public:

    ///
    /// \brief Construct a wrapping sphere
    ///
    WrappingSphere();

    ///
    /// \brief Construct a wrapping sphere
    /// \param x X-Component of the sphere
    /// \param y Y-Component of the sphere
    /// \param z Z-Component of the sphere
    /// \param diameter Diameter of the sphere
    ///
    WrappingSphere(
        const utils::Scalar& x,
        const utils::Scalar& y,
        const utils::Scalar& z,
        const utils::Scalar& diameter);

    ///
    /// \brief Construct a wrapping sphere
    /// \param x X-Component of the sphere
    /// \param y Y-Component of the sphere
    /// \param z Z-Component of the sphere
    /// \param diameter Diameter of the sphere
    /// \param name Name of the sphere
    /// \param parentName Name of the parent segment
    ///
    WrappingSphere(
        const utils::Scalar& x,
        const utils::Scalar& y,
        const utils::Scalar& z,
        const utils::Scalar& diameter,
        const utils::String &name,
        const utils::String &parentName);

    ///
    /// \brief Construct a wrapping sphere
    /// \param v Position of the center of the sphere
    /// \param diameter Diameter of the sphere
    ///
    WrappingSphere(
        const utils::Vector3d &v,
        const utils::Scalar& diameter);

    ///
    /// \brief Deep copy of the wrapping sphere
    /// \return A deep copy of the wrapping sphere
    ///
    WrappingSphere DeepCopy() const;

    ///
    /// \brief Deep copy of the wrapping sphere in another wrapping sphere
    /// \param other The wrapping sphere to copy
    ///
    void DeepCopy(
        const WrappingSphere& other);

    ///
    /// \brief Not yet implemented
    ///
    virtual void wrapPoints(
        const utils::RotoTrans&,
        const utils::Vector3d&,
        const utils::Vector3d&,
        utils::Vector3d&,
        utils::Vector3d&,
        utils::Scalar* = nullptr) {}

    ///
    /// \brief Not yet implemented
    ///
    virtual void wrapPoints(
        rigidbody::Joints&,
        const rigidbody::GeneralizedCoordinates&,
        const utils::Vector3d&,
        const utils::Vector3d&,
        utils::Vector3d&,
        utils::Vector3d&,
        utils::Scalar* = nullptr) {}

    ///
    /// \brief Not yet implemented
    ///
    virtual void wrapPoints(
        utils::Vector3d&,
        utils::Vector3d&,
        utils::Scalar* = nullptr) {}

    ///
    /// \brief Return the RotoTrans matrix of the sphere
    /// \param model The joint model
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics should be computed
    /// \return The RotoTrans matrix of the sphere
    ///
    const utils::RotoTrans& RT(
        rigidbody::Joints &model,
        const rigidbody::GeneralizedCoordinates&Q ,
        bool updateKin  = true);

    ///
    /// \brief Set the diameter of the wrappping sphere
    /// \param val Value of the diameter
    ///
    void setDiameter(
        const utils::Scalar& val);

    ///
    /// \brief Return the diameter of the wrapping sphere
    /// \return The diameter of the wrapping sphere
    ///
    const utils::Scalar& diameter() const;

protected:
    std::shared_ptr<utils::Scalar>
    m_dia; ///< Diameter of the wrapping sphere

};

}
}

#endif // BIORBD_MUSCLES_WRAPPING_SPHERE_H
