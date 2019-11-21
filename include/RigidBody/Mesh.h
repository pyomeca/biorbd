#ifndef BIORBD_RIGIDBODY_MESH_H
#define BIORBD_RIGIDBODY_MESH_H

#include <memory>
#include <vector>
#include "Eigen/Dense"
#include "biorbdConfig.h"

namespace biorbd {
namespace utils {
class Vector3d;
class Path;
}

namespace rigidbody {
class Patch;

///
/// \brief Class Mesh
///
class BIORBD_API Mesh
{
public:
    ///
    /// \brief Construct mesh
    ///
    Mesh();

    ///
    /// \brief Construct mesh
    /// \brief mesh 3D vector
    ///
    Mesh(
            const std::vector<biorbd::utils::Vector3d>& mesh);

    ///
    /// \brief Construct mesh
    /// \param mesh 3D vector
    /// \param v Patch
    ///
   Mesh(
            const std::vector<biorbd::utils::Vector3d>& mesh,
            const std::vector<biorbd::rigidbody::Patch>&v);

    ///
    /// \brief Deep copy of the mesh
    /// \return A copy of mesh
    ///
    biorbd::rigidbody::Mesh DeepCopy() const;

    ///
    /// \brief Deep copy of the mesh
    /// \param other The mesh to copy
    ///
    void DeepCopy(const biorbd::rigidbody::Mesh& other);

    // Related to the mesh points

    ///
    /// \brief Add a point to the mesh
    /// \param node The point to add
    ///
    void addPoint(const biorbd::utils::Vector3d & node);

    ///
    /// \brief Return a point at a specific position
    /// \param i Position
    /// \return A point at position i
    ///
    const biorbd::utils::Vector3d& point(unsigned int i) const;

    ///
    /// \brief Returns the size of the mesh
    /// \return The size of the mesh
    ///
    unsigned int size() const;

    // Related to the patches

    ///
    /// \brief Add a patch to the mesh
    /// \param v The patch to add
    ///
    void addPatch(const biorbd::rigidbody::Patch &v);

    ///
    /// \brief Add a patch to the mesh
    /// \param v The patch to add (format Eigen vector)
    ///
    void addPatch(const Eigen::Vector3i &v);

    ///
    /// \brief Return the patches of the mesh
    /// \return The patches of the mesh
    ///
    const std::vector<biorbd::rigidbody::Patch>& patch() const;

    ///
    /// \brief Return the patch of the mesh at a specified position
    /// \param i Position
    /// \return The patch at position i
    ///
    const biorbd::rigidbody::Patch& patch(unsigned int i) const;

    ///
    /// \brief Return the size of the patch
    /// \return The size of the patch
    ///
    unsigned int nbPatch();

    
    ///
    /// \brief Set the path of the underlying mesh file
    /// \param path Path for the mesh file
    ///
    void setPath(const biorbd::utils::Path& path);

    ///
    /// \brief Return the path of the mesh file
    /// \return The path of the mesh file
    ///
    const biorbd::utils::Path& path() const;

protected:
    std::shared_ptr<std::vector<biorbd::utils::Vector3d>> m_vertex; ///< The vertex
    std::shared_ptr<std::vector<biorbd::rigidbody::Patch>> m_patch; ///< The patch
    std::shared_ptr<biorbd::utils::Path> m_pathFile; ///< The path to the mesh file
};

}}

#endif // BIORBD_RIGIDBODY_MESH_H
