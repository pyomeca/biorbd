#ifndef BIORBD_RIGIDBODY_PATCH_H
#define BIORBD_RIGIDBODY_PATCH_H

#include <memory>
#include <Eigen/Dense>
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class Node3d;
}

namespace rigidbody {

///
/// \brief Class Patch
///
class BIORBD_API Patch
{
public:

    ///
    /// \brief Contruct patch
    /// \param vertex The vertex that connect to form a patch
    ///
    Patch(const Eigen::Vector3i& vertex= Eigen::Vector3i());

    ///
    /// \brief Deep copy of a patch
    /// \return A deep copy of a patch
    ///
    biorbd::rigidbody::Patch DeepCopy() const;

    ///
    /// \brief Deep copy of a patch into another one
    /// \param other The patch to copy
    ///
    void DeepCopy(const biorbd::rigidbody::Patch& other);

    ///
    /// \brief  TODO
    /// \param i TODO
    ///
    int &operator() (int i);

    ///
    /// \brief Construct patch with new points TODO
    /// \param pts The new points to construct patch TODO
    ///
    void patch(const Eigen::Vector3i& pts);

    ///
    /// \brief Construct patch with another patch TODO
    /// \param v The other patch TODO
    ///
    void patch(const Patch&v);

    biorbd::utils::Node3d patchAsDouble(); // retourne les patchs
protected:
    std::shared_ptr<Eigen::Vector3i> m_patch; ///< Patch

};

}}

#endif // BIORBD_RIGIDBODY_PATCH_H
