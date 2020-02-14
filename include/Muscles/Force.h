#ifndef BIORBD_MUSCLES_FORCE_H
#define BIORBD_MUSCLES_FORCE_H

#include <memory>
#include "biorbdConfig.h"

#include "Utils/Vector3d.h"

namespace biorbd {

namespace muscles {
class Geometry;
///
/// \brief Class Force 
///
class BIORBD_API Force : public biorbd::utils::Vector3d
{
public:
    ///
    /// \brief Construct force
    ///
    Force();

    ///
    /// \brief Construct force
    /// \param x X-component of the force
    /// \param y Y-component of the force
    /// \param z Z-component of the force
    /// 
    Force(
            double x,
            double y,
            double z);

    ///
    /// \brief Construct force from another force
    /// \param other The other force
    ///
    Force(
            const biorbd::muscles::Force& other);

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    Force(
            const RigidBodyDynamics::Math::Vector3d& v);

    ///
    /// \brief Construct force from another force
    /// \param other Force in Vector3d format
    ///
    Force(
            const biorbd::utils::Vector3d& other);

    ///
    /// \brief Construct force
    /// \param geo The geometry of the muscle
    /// \param norm The norm of the force
    ///
    Force(
            const biorbd::muscles::Geometry& geo,
            double norm);

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct force from another force
    /// \param other The other force in vector form
    ///
    template<typename OtherDerived> Force(
            const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Vector3d(other){}
#endif

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Force();

    ///
    /// \brief Deep copy of a force
    /// \return A deep copy of the force
    ///
    biorbd::muscles::Force DeepCopy() const;

    ///
    /// \brief Deep copy of a force
    /// \param other The force to copy
    ///
    void DeepCopy(
            const biorbd::muscles::Force& other);

    ///
    /// \brief Set the force from the muscle geometry
    /// \param geo The muscle geometry
    /// \param norm The norm of the force
    ///
    virtual void setForceFromMuscleGeometry(
            const biorbd::muscles::Geometry& geo,
            biorbd::utils::Scalar norm);

#ifdef BIORBD_USE_EIGEN3_MATH

    ///
    /// \brief Equal operator to be used with another force
    /// \param other The eigen matrix of force
    /// \return The force
    ///
    template<typename OtherDerived>
        biorbd::muscles::Force& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->Eigen::Vector3d::operator=(other);
            return *this;
        }

#endif

#ifndef SWIG
    ///
    /// \brief Equal operator to be used with another force
    /// \param other Other force to copy
    /// \return The new force
    ///
    biorbd::muscles::Force& operator=(
            const biorbd::muscles::Force& other);
#endif

};

}}

#endif // BIORBD_MUSCLES_FORCE_H
