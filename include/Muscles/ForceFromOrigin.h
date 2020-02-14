#ifndef BIORBD_MUSCLES_FORCE_FROM_ORIGIN_H
#define BIORBD_MUSCLES_FORCE_FROM_ORIGIN_H

#include "biorbdConfig.h"
#include "Muscles/Force.h"

namespace biorbd {
namespace muscles {

///
/// \brief Class Force in the muscle from the origin perspective
///
class BIORBD_API ForceFromOrigin : public biorbd::muscles::Force
{
public:
    ///
    /// \brief Construct force
    /// \param x X-component of the force
    /// \param y Y-component of the force
    /// \param z Z-component of the force
    ///
    ForceFromOrigin(
            double x = 0,
            double y = 0,
            double z = 0);

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    ForceFromOrigin(
            const RigidBodyDynamics::Math::Vector3d& v);

    ///
    /// \brief Construct a force from origin
    /// \param geo Geometry of the muscle
    /// \param norm The norm of the force
    ///
    ForceFromOrigin(
            const biorbd::muscles::Geometry& geo,
            double norm);

#ifdef BIORBD_USE_EIGEN3_MATH

    ///
    /// \brief Construct a force from origin from an Eigen matrix of force
    /// \param other The eigen matrix of force
    ///
    template<typename OtherDerived> ForceFromOrigin(
            const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::muscles::Force(other){}

#endif

    ///
    /// \brief Deep copy of the force from origin
    /// \return A deep copy of the force from origin
    ///
    biorbd::muscles::ForceFromOrigin DeepCopy() const;

    ///
    /// \brief Deep copy of another force from origin
    /// \param other The force from origin to copy
    ///
    void DeepCopy(const biorbd::muscles::ForceFromOrigin& other);

    ///
    /// \brief Set the force from muscle geometry
    /// \param geo THe muscle geometry
    /// \param norm The norm of the force
    ///
    virtual void setForceFromMuscleGeometry(
            const biorbd::muscles::Geometry& geo,
            biorbd::utils::Scalar norm);

#ifdef BIORBD_USE_EIGEN3_MATH

    ///
    /// \brief Equal operator to be used with an Eigen matrix to construct force from origin
    /// \param other The eigen matrix of force from origin
    /// \return The force from origin
    ///
    template<typename OtherDerived>
    biorbd::muscles::ForceFromOrigin& operator=(
            const Eigen::MatrixBase <OtherDerived>& other){
        this->biorbd::muscles::Force::operator=(other);
        return *this;
    }

#endif

};

}}

#endif // BIORBD_MUSCLES_FORCE_FROM_ORIGIN_H
