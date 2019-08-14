#ifndef S2M_BONE_MESH_H
#define S2M_BONE_MESH_H

#include <vector>
#include "Eigen/Dense"
#include "biorbdConfig.h"
#include "Utils/Path.h"

namespace biorbd { namespace utils {
class Node;
}}
class s2mPatch;
class BIORBD_API s2mBoneMesh
{
public:
    s2mBoneMesh(
            const std::vector<biorbd::utils::Node>& = std::vector<biorbd::utils::Node>(),
            const std::vector<s2mPatch>& = std::vector<s2mPatch>());
    virtual ~s2mBoneMesh();

    // Concernant les points du mesh
    void addPoint(const biorbd::utils::Node &);
    const biorbd::utils::Node& point(unsigned int i) const;
    unsigned int size() const;

    void setPath(const biorbd::utils::Path& path);
    const biorbd::utils::Path& path() const;

    // Concernant les patch
    void addPatch(const s2mPatch &);
    void addPatch(const Eigen::Vector3i &);
    const std::vector<s2mPatch>& patch() const;
    const s2mPatch& patch(unsigned int i) const;
    unsigned int sizePatch();

protected:
    biorbd::utils::Path m_pathFile;
    std::vector<biorbd::utils::Node> m_mesh;
    std::vector<s2mPatch> m_patch;

};

#endif // S2M_BONE_MESH_H
