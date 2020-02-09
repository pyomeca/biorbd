#ifndef BIORBD_MUSCLES_FORCE_FROM_INSERTION_H
#define BIORBD_MUSCLES_FORCE_FROM_INSERTION_H

#include "biorbdConfig.h"
#include "Muscles/Force.h"

namespace biorbd {
namespace muscles {

///
/// \brief Class Force in the muscle from the insertion perspective
///
class BIORBD_API ForceFromInsertion : public biorbd::muscles::Force
{
public:
    ///
    /// \brief Construct force
    /// \param x X-component of the force
    /// \param y Y-component of the force
    /// \param z Z-component of the force
    ///
    ForceFromInsertion(
            double x = 0,
            double y = 0,
            double z = 0);

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct force from insertion from Eiggen matrix
    /// \param other The Eigen force matrix
    ///
    template<typename OtherDerived> ForceFromInsertion(
            const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::muscles::Force(other){}
#endif
#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    ForceFromInsertion(
            const RigidBodyDynamics::Math::Vector3d& v);

#endif

    ///
    /// \brief Construct force from insertion
    /// \param geo The muscle geometry
    /// \param vectorNorm The norm of the vector
    ///
    ForceFromInsertion(
            const biorbd::muscles::Geometry& geo,
            double vectorNorm);

    ///
    /// \brief Deep copy of the force from insertion
    /// \return Deep copy ofo the force from insertion
    ///
    biorbd::muscles::ForceFromInsertion DeepCopy() const;

    ///
    /// \brief Deep copy of the force from insertion from another force from insertion
    /// \param other The other force from insertion
    ///
    void DeepCopy(const biorbd::muscles::ForceFromInsertion& other);

    // Get et set

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
    /// \brief Equal operator to be used with an Eigen matrix to construct force from insertion
    /// \param other The eigen matrix of force from insertion
    /// \return The force from insertion
    ///
    template<typename OtherDerived>
    biorbd::muscles::ForceFromInsertion& operator=(
            const Eigen::MatrixBase <OtherDerived>& other){
        this->biorbd::muscles::Force::operator=(other);
        return *this;
    }
#endif
};

}}

#endif // BIORBD_MUSCLES_FORCE_FROM_INSERTION_H
