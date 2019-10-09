#define BIORBD_API_EXPORTS
#include "RigidBody/BoneMesh.h"

#include "Utils/Path.h"
#include "Utils/Node3d.h"
#include "RigidBody/Patch.h"

biorbd::rigidbody::BoneMesh::BoneMesh() :
    m_mesh(std::make_shared<std::vector<biorbd::utils::Node3d>>()),
    m_patch(std::make_shared<std::vector<biorbd::rigidbody::Patch>>()),
    m_pathFile(std::make_shared<biorbd::utils::Path>())
{

}

biorbd::rigidbody::BoneMesh::BoneMesh(
        const std::vector<biorbd::utils::Node3d> &mesh) :
    m_mesh(std::make_shared<std::vector<biorbd::utils::Node3d>>(mesh)),
    m_patch(std::make_shared<std::vector<biorbd::rigidbody::Patch>>()),
    m_pathFile(std::make_shared<biorbd::utils::Path>())
{

}

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
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::BoneMesh::DeepCopy(const biorbd::rigidbody::BoneMesh &other)
{
    m_mesh->resize(other.m_mesh->size());
    for (unsigned int i=0; i<other.m_mesh->size(); ++i)
        (*m_mesh)[i] = (*other.m_mesh)[i].DeepCopy();
    m_patch->resize(other.m_patch->size());
    for (unsigned int i=0; i<other.m_patch->size(); ++i)
        (*m_patch)[i] = (*other.m_patch)[i].DeepCopy();
    *m_pathFile = other.m_pathFile->DeepCopy();
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
