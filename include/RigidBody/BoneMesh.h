#ifndef BIORBD_RIGIDBODY_BONE_MESH_H
#define BIORBD_RIGIDBODY_BONE_MESH_H

#include <vector>
#include "Eigen/Dense"
#include "biorbdConfig.h"
#include "Utils/Path.h"

namespace biorbd { namespace utils {
class Node;
}}

namespace biorbd { namespace rigidbody {
class Patch;

class BIORBD_API Mesh
{
public:
    Mesh(
            const std::vector<biorbd::utils::Node>& = std::vector<biorbd::utils::Node>(),
            const std::vector<biorbd::rigidbody::Patch>& = std::vector<biorbd::rigidbody::Patch>());
    virtual ~Mesh();

    // Concernant les points du mesh
    void addPoint(const biorbd::utils::Node &);
    const biorbd::utils::Node& point(unsigned int i) const;
    unsigned int size() const;

    void setPath(const biorbd::utils::Path& path);
    const biorbd::utils::Path& path() const;

    // Concernant les patch
    void addPatch(const biorbd::rigidbody::Patch &);
    void addPatch(const Eigen::Vector3i &);
    const std::vector<biorbd::rigidbody::Patch>& patch() const;
    const biorbd::rigidbody::Patch& patch(unsigned int i) const;
    unsigned int sizePatch();

protected:
    biorbd::utils::Path m_pathFile;
    std::vector<biorbd::utils::Node> m_mesh;
    std::vector<biorbd::rigidbody::Patch> m_patch;

};

}}

#endif // BIORBD_RIGIDBODY_BONE_MESH_H
