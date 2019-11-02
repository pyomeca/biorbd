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

class BIORBD_API Patch
{
public:
    Patch(const Eigen::Vector3i& = Eigen::Vector3i());
    biorbd::rigidbody::Patch DeepCopy() const;
    void DeepCopy(const biorbd::rigidbody::Patch& other);

    // Ajouter un patch au lot
    int &operator() (int);
    void patch(const Eigen::Vector3i&);
    void patch(const Patch&);
    biorbd::utils::Node3d patchAsDouble(); // retourne les patchs
protected:
    std::shared_ptr<Eigen::Vector3i> m_patch;

};

}}

#endif // BIORBD_RIGIDBODY_PATCH_H
