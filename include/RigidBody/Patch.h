#ifndef BIORBD_RIGIDBODY_PATCH_H
#define BIORBD_RIGIDBODY_PATCH_H

#include <Eigen/Dense>
#include "biorbdConfig.h"

namespace biorbd {
namespace rigidbody {

class BIORBD_API Patch
{
public:
    Patch(const Eigen::Vector3i& = Eigen::Vector3i());

    // Ajouter un patch au lot
    int &operator() (int);
    void patch(const Eigen::Vector3i&);
    void patch(const Patch&);
    Patch patch(); // retourne les patchs
protected:
    Eigen::Vector3i m_patch;

};

}}

#endif // BIORBD_RIGIDBODY_PATCH_H