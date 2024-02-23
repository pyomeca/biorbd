#define BIORBD_API_EXPORTS
#include "RigidBody/Mesh.h"

#include "Utils/Path.h"
#include "Utils/Vector3d.h"
#include "Utils/RotoTrans.h"
#include "RigidBody/MeshFace.h"

using namespace BIORBD_NAMESPACE;

rigidbody::Mesh::Mesh() :
    m_rotation(std::make_shared<utils::RotoTrans>()),
    m_vertex(std::make_shared<std::vector<utils::Vector3d>>()),
    m_faces(std::make_shared<std::vector<rigidbody::MeshFace>>()),
    m_pathFile(std::make_shared<utils::Path>()),
    m_patchColor(std::make_shared<utils::Vector3d>(0.89, 0.855, 0.788)),
    m_scale(std::make_shared<utils::Vector3d>(1.0, 1.0, 1.0))
{

}

rigidbody::Mesh::Mesh(
        const std::vector<utils::Vector3d> &other):
    m_rotation(std::make_shared<utils::RotoTrans>()),
    m_vertex(std::make_shared<std::vector<utils::Vector3d>>(other)),
    m_faces(std::make_shared<std::vector<rigidbody::MeshFace>>()),
    m_pathFile(std::make_shared<utils::Path>()),
	m_patchColor(std::make_shared<utils::Vector3d>(0.89, 0.855, 0.788)),
	m_scale(std::make_shared<utils::Vector3d>(1.0, 1.0, 1.0))
{

}

rigidbody::Mesh::Mesh(
        const std::vector<utils::Vector3d> &vertex,
        const std::vector<rigidbody::MeshFace> & faces) :
    m_rotation(std::make_shared<utils::RotoTrans>()),
    m_vertex(std::make_shared<std::vector<utils::Vector3d>>(vertex)),
    m_faces(std::make_shared<std::vector<rigidbody::MeshFace>>(faces)),
    m_pathFile(std::make_shared<utils::Path>()),
	m_patchColor(std::make_shared<utils::Vector3d>(0.89, 0.855, 0.788)),
	m_scale(std::make_shared<utils::Vector3d>(1.0, 1.0, 1.0))
{

}

rigidbody::Mesh rigidbody::Mesh::DeepCopy() const
{
    rigidbody::Mesh copy;
    copy.DeepCopy(*this);
    return copy;
}

void rigidbody::Mesh::DeepCopy(const rigidbody::Mesh &other)
{
    m_vertex->resize(other.m_vertex->size());
    for (size_t i=0; i<other.m_vertex->size(); ++i) {
        (*m_vertex)[i] = (*other.m_vertex)[i].DeepCopy();
    }
    m_faces->resize(other.m_faces->size());
    for (size_t i=0; i<other.m_faces->size(); ++i) {
        (*m_faces)[i] = (*other.m_faces)[i].DeepCopy();
    }
    *m_pathFile = other.m_pathFile->DeepCopy();
    *m_patchColor = other.m_patchColor->DeepCopy();
    *m_rotation = *other.m_rotation;
    *m_rotation = *other.m_rotation;
}

void rigidbody::Mesh::setColor(
        const utils::Vector3d &color)
{
    *m_patchColor = color;
}

utils::Vector3d &rigidbody::Mesh::color() const
{
    return *m_patchColor;
}

bool rigidbody::Mesh::hasMesh() const {
    return m_vertex->size() > 0;
}
void rigidbody::Mesh::addPoint(const utils::Vector3d &node)
{
    m_vertex->push_back(node);
}
const utils::Vector3d &rigidbody::Mesh::point(
    size_t idx) const
{
    return (*m_vertex)[idx];
}
size_t rigidbody::Mesh::nbVertex() const
{
    return m_vertex->size();
}

void rigidbody::Mesh::rotate(
        const utils::RotoTrans &rt)
{
    *m_rotation = rt;
    for (auto& v : *m_vertex){
        v.applyRT(rt);
    }
}

utils::RotoTrans &rigidbody::Mesh::getRotation() const
{
    return *m_rotation;
}

void rigidbody::Mesh::scale(
        const utils::Vector3d &scaler)
{   
    *m_scale = scaler;
    for (auto& v: *m_vertex){
        v(0) *= scaler(0);
        v(1) *= scaler(1);
        v(2) *= scaler(2);
    }
}

utils::Vector3d &rigidbody::Mesh::getScale() const
{   
    return *m_scale;
}

size_t rigidbody::Mesh::nbFaces()
{
    return m_faces->size();
}
void rigidbody::Mesh::addFace(const rigidbody::MeshFace& face)
{
    m_faces->push_back(face);
}
void rigidbody::Mesh::addFace(const std::vector<int> & face)
{
    addFace(rigidbody::MeshFace(face));
}
const std::vector<rigidbody::MeshFace>& rigidbody::Mesh::faces()
const
{
    return *m_faces;
}
const rigidbody::MeshFace &rigidbody::Mesh::face(
    size_t idx) const
{
    return (*m_faces)[idx];
}

void rigidbody::Mesh::setPath(const utils::Path& path)
{
    *m_pathFile = path;
}

const utils::Path &rigidbody::Mesh::path() const
{
    return *m_pathFile;
}
