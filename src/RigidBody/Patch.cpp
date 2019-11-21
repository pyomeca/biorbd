#define BIORBD_API_EXPORTS
#include "RigidBody/Patch.h"

#include "Utils/Vector3d.h"

biorbd::rigidbody::Patch::Patch(const Eigen::Vector3i& vertex) :
    m_patch(std::make_shared<Eigen::Vector3i>(vertex))
{

}

biorbd::rigidbody::Patch biorbd::rigidbody::Patch::DeepCopy() const
{
    biorbd::rigidbody::Patch copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::Patch::DeepCopy(const biorbd::rigidbody::Patch &other)
{
    *m_patch = *other.m_patch;
}

int &biorbd::rigidbody::Patch::operator()(int i)
{
    return (*m_patch)[i];
}

biorbd::utils::Vector3d biorbd::rigidbody::Patch::patchAsDouble()
{
    return biorbd::utils::Vector3d(static_cast<double>(m_patch->x()),
                                 static_cast<double>(m_patch->y()),
                                 static_cast<double>(m_patch->z()));
}

void biorbd::rigidbody::Patch::patch(const Eigen::Vector3i & pts)
{
    *m_patch = pts;
}

void biorbd::rigidbody::Patch::patch(const biorbd::rigidbody::Patch &v)
{
    m_patch = v.m_patch;
}

