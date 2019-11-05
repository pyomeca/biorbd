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
    /// \brief Construct force
    /// \param x Position on X axis
    /// \param y Position on Y axis
    /// \param z Position on Z axis
    /// 
    Force(
            const biorbd::muscles::Force& force);
    template<typename OtherDerived> Force(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Vector3d(other){}
    Force(
            const biorbd::utils::Node3d& force);
    Force(
            const biorbd::muscles::Geometry& geo,
            double vectorNorm);
    virtual ~Force();
    biorbd::muscles::Force DeepCopy() const;
    void DeepCopy(const biorbd::muscles::Force& other);

    // Get et set
    virtual void setForceFromMuscleGeometry(
            const biorbd::muscles::Geometry& geo,
            double vectorNorm);

    template<typename OtherDerived>
        biorbd::muscles::Force& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->Eigen::Vector3d::operator=(other);
            return *this;
        }
    biorbd::muscles::Force& operator=(const biorbd::muscles::Force& other);
};

}}

#endif // BIORBD_MUSCLES_FORCE_H
