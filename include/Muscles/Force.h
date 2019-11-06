#ifndef BIORBD_MUSCLES_FORCE_H
#define BIORBD_MUSCLES_FORCE_H

#include <memory>
#include <Eigen/Dense>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class Node3d;
}

namespace muscles {
class Geometry;
///
/// \brief Class Force 
///
class BIORBD_API Force : public Eigen::Vector3d
{
public:
    ///
    /// \brief Construct force
    ///
    Force();
    ///
    /// \brief Construct force
    /// \param x Position on X axis
    /// \param y Position on Y axis
    /// \param z Position on Z axis
    /// 
    Force(
            double x,
            double y,
            double z);

    ///
    /// \brief Construct force from another force
    /// \param force The other force
    ///
    Force(
            const biorbd::muscles::Force& force);

    ///
    /// \brief Construct force from another force
    /// \param other The other force in vector form
    ///
    template<typename OtherDerived> Force(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Vector3d(other){}

    ///
    /// \brief Construct force from another force
    /// \param force Force in Node3D format
    ///
    Force(
            const biorbd::utils::Node3d& force);

    ///
    /// \brief Construct force
    /// \param geo The geometry of the muscle
    /// \param vectorNorm The norm of the vector
    ///
    Force(
            const biorbd::muscles::Geometry& geo,
            double vectorNorm);

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
    void DeepCopy(const biorbd::muscles::Force& other);


    ///
    /// \brief Set the force from the muscle geometry
    /// \param geo The muscle geometry
    /// \param vectorNorm The norm of the vector
    ///
    virtual void setForceFromMuscleGeometry(
            const biorbd::muscles::Geometry& geo,
            double vectorNorm);

    ///
    /// \brief Equal operator to be used with an Eigen matrix to construct force
    /// \param other The eigen matrix of force
    /// \return The force
    ///
    template<typename OtherDerived>
        biorbd::muscles::Force& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->Eigen::Vector3d::operator=(other);
            return *this;
        }

    ///
    /// \brief Equal operator to be used with another force
    /// \param other Other force to copy
    /// \return The new force
    ///
    biorbd::muscles::Force& operator=(const biorbd::muscles::Force& other);
};

}}

#endif // BIORBD_MUSCLES_FORCE_H
