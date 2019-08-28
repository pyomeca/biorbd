#define BIORBD_API_EXPORTS
#include "RigidBody/BoneMesh.h"

#include "Utils/Path.h"
#include "Utils/Node3d.h"
#include "RigidBody/Patch.h"

biorbd::rigidbody::BoneMesh::BoneMesh(
        const std::vector<biorbd::utils::Node3d> &mesh,
        const std::vector<biorbd::rigidbody::Patch> & v) :
    m_mesh(std::make_shared<std::vector<biorbd::utils::Node3d>>(mesh)),
    m_patch(std::make_shared<std::vector<biorbd::rigidbody::Patch>>(v)),
    m_pathFile(std::make_shared<biorbd::utils::Path>())
{

}

biorbd::rigidbody::BoneMesh biorbd::rigidbody::BoneMesh::DeepCopy() const
{
    biorbd::rigidbody::BoneMesh copy;
    *copy.m_mesh = *m_mesh;
    *copy.m_patch = *m_patch;
    *copy.m_pathFile = *m_pathFile;
    return copy;
}

void biorbd::rigidbody::BoneMesh::DeepCopy(const biorbd::rigidbody::BoneMesh &other)
{
    m_mesh = std::make_shared<std::vector<biorbd::utils::Node3d>>(*other.m_mesh);
    m_patch = std::make_shared<std::vector<biorbd::rigidbody::Patch>>(*other.m_patch);
    m_pathFile = std::make_shared<biorbd::utils::Path>(*other.m_pathFile);
}

void biorbd::rigidbody::BoneMesh::addPoint(const biorbd::utils::Node3d &node)
{
    m_mesh->push_back(node);
}
const biorbd::utils::Node3d &biorbd::rigidbody::BoneMesh::point(unsigned int i) const
{
    return (*m_mesh)[i];
}
unsigned int biorbd::rigidbody::BoneMesh::size() const
{
    return static_cast<unsigned int>(m_mesh->size());
}

unsigned int biorbd::rigidbody::BoneMesh::sizePatch()
{
    return static_cast<unsigned int>(m_patch->size());
}
void biorbd::rigidbody::BoneMesh::addPatch(const biorbd::rigidbody::Patch & v)
{
    m_patch->push_back(v);
}
void biorbd::rigidbody::BoneMesh::addPatch(const Eigen::Vector3i & v)
{
    addPatch(biorbd::rigidbody::Patch(v));
}
const std::vector<biorbd::rigidbody::Patch>& biorbd::rigidbody::BoneMesh::patch() const
{
    return *m_patch;
}
const biorbd::rigidbody::Patch &biorbd::rigidbody::BoneMesh::patch(unsigned int i) const
{
    return (*m_patch)[i];
}

void biorbd::rigidbody::BoneMesh::setPath(const biorbd::utils::Path& path)
{
    *m_pathFile = path;
}

const biorbd::utils::Path &biorbd::rigidbody::BoneMesh::path() const
{
    return *m_pathFile;
}
