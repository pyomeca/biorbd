#define BIORBD_API_EXPORTS
#include "RigidBody/MeshFace.h"

#include "Utils/Vector3d.h"

using namespace BIORBD_NAMESPACE;

rigidbody::MeshFace::MeshFace(const std::vector<int> &vertex)
    : m_face(std::make_shared<std::vector<int>>(vertex)) {}

rigidbody::MeshFace rigidbody::MeshFace::DeepCopy() const {
  rigidbody::MeshFace copy;
  copy.DeepCopy(*this);
  return copy;
}

void rigidbody::MeshFace::DeepCopy(const rigidbody::MeshFace &other) {
  *m_face = *other.m_face;
}

int &rigidbody::MeshFace::operator()(size_t idx) { return (*m_face)[idx]; }

utils::Vector3d rigidbody::MeshFace::faceAsDouble() {
  return utils::Vector3d(
      static_cast<double>((*m_face)[0]),
      static_cast<double>((*m_face)[1]),
      static_cast<double>((*m_face)[2]));
}

std::vector<int> rigidbody::MeshFace::face() { return *m_face; }

void rigidbody::MeshFace::setFace(const std::vector<int> &pts) {
  *m_face = pts;
}

void rigidbody::MeshFace::setFace(const rigidbody::MeshFace &other) {
  m_face = other.m_face;
}
