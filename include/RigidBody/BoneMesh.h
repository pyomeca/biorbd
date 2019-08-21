#ifndef BIORBD_RIGIDBODY_BONE_MESH_H
#define BIORBD_RIGIDBODY_BONE_MESH_H

#include <memory>
#include <vector>
#include "Eigen/Dense"
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class Node3d;
class Path;
}

namespace rigidbody {
class Patch;

class BIORBD_API BoneMesh
{
public:
    BoneMesh(
            const std::vector<biorbd::utils::Node3d>& = std::vector<biorbd::utils::Node3d>(),
            const std::vector<biorbd::rigidbody::Patch>& = std::vector<biorbd::rigidbody::Patch>());
    biorbd::rigidbody::BoneMesh DeepCopy();

    // Concernant les points du mesh
    void addPoint(const biorbd::utils::Node3d &);
    const biorbd::utils::Node3d& point(unsigned int i) const;
    unsigned int size() const;

    // Concernant les patch
    void addPatch(const biorbd::rigidbody::Patch &);
    void addPatch(const Eigen::Vector3i &);
    const std::vector<biorbd::rigidbody::Patch>& patch() const;
    const biorbd::rigidbody::Patch& patch(unsigned int i) const;
    unsigned int sizePatch();

    // Path of the underlying mesh file
    void setPath(const biorbd::utils::Path& path);
    const biorbd::utils::Path& path() const;

protected:
    std::vector<biorbd::utils::Node3d> m_mesh;
    std::vector<biorbd::rigidbody::Patch> m_patch;
    std::shared_ptr<biorbd::utils::Path> m_pathFile;
};

}}

#endif // BIORBD_RIGIDBODY_BONE_MESH_H
