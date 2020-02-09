#ifndef BIORBD_MUSCLES_WRAPPING_SPHERE_H
#define BIORBD_MUSCLES_WRAPPING_SPHERE_H

#include "biorbdConfig.h"
#include "Muscles/WrappingObject.h"

namespace biorbd {
namespace muscles {
///
/// \brief Sphere object that makes the muscle to wrap around
///
class BIORBD_API WrappingSphere : public biorbd::muscles::WrappingObject
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
            double x,
            double y,
            double z,
            double diameter);

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
            double x,
            double y,
            double z,
            double diameter, 
            const biorbd::utils::String &name, 
            const biorbd::utils::String &parentName);

    ///
    /// \brief Construct a wrapping sphere
    /// \param v Position of the center of the sphere
    /// \param diameter Diameter of the sphere
    ///
    WrappingSphere(
            const biorbd::utils::Vector3d &v, 
            double diameter);

    ///
    /// \brief Deep copy of the wrapping sphere
    /// \return A deep copy of the wrapping sphere
    ///
    biorbd::muscles::WrappingSphere DeepCopy() const;

    ///
    /// \brief Deep copy of the wrapping sphere in another wrapping sphere
    /// \param other The wrapping sphere to copy
    ///
    void DeepCopy(
            const biorbd::muscles::WrappingSphere& other);

    ///
    /// \brief Not yet implemented
    ///
    virtual void wrapPoints(
            const biorbd::utils::RotoTrans&,
            const biorbd::utils::Vector3d&,
            const biorbd::utils::Vector3d&,
            biorbd::utils::Vector3d&,
            biorbd::utils::Vector3d&,
            biorbd::utils::Scalar* = nullptr) {}

    ///
    /// \brief Not yet implemented
    ///
    virtual void wrapPoints(
            biorbd::rigidbody::Joints&,
            const biorbd::rigidbody::GeneralizedCoordinates&,
            const biorbd::utils::Vector3d&,
            const biorbd::utils::Vector3d&,
            biorbd::utils::Vector3d&,
            biorbd::utils::Vector3d&,
            biorbd::utils::Scalar* = nullptr) {}

    ///
    /// \brief Not yet implemented
    ///
    virtual void wrapPoints(
            biorbd::utils::Vector3d&,
            biorbd::utils::Vector3d&,
            biorbd::utils::Scalar* = nullptr) {}

    ///
    /// \brief Return the RotoTrans matrix of the sphere
    /// \param model The joint model
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics should be computed
    /// \return The RotoTrans matrix of the sphere
    ///
    const biorbd::utils::RotoTrans& RT(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates&Q ,
            bool updateKin  = true);

    ///
    /// \brief Set the diameter of the wrappping sphere
    /// \param val Value of the diameter
    ///
    void setDiameter(double val);

    ///
    /// \brief Return the diameter of the wrapping sphere
    /// \return The diameter of the wrapping sphere
    ///
    double diameter() const;

protected:
    std::shared_ptr<double> m_dia; ///< Diameter of the wrapping sphere

};

}}

#endif // BIORBD_MUSCLES_WRAPPING_SPHERE_H
