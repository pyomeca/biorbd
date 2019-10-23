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

///
/// \brief Class BoneMesh
///
class BIORBD_API BoneMesh
{
public:
    ///
    /// \brief Construct bone mesh
    ///
    BoneMesh();

    ///
    /// \brief Construct bone mesh
    /// \brief mesh Nodes
    ///
    BoneMesh(
            const std::vector<biorbd::utils::Node3d>& mesh);

    ///
    /// \brief Construct bone mesh
    /// \param mesh Nodes
    /// \param v Patch
    ///
    BoneMesh(
            const std::vector<biorbd::utils::Node3d>& mesh,
            const std::vector<biorbd::rigidbody::Patch>&v);

    ///
    /// \brief Deep copy of the bone mesh
    /// \return A copy of bone mesh
    ///
    biorbd::rigidbody::BoneMesh DeepCopy() const;

    ///
    /// \brief Deep copy of the bone mesh
    /// \param other The mesh to copy
    ///
    void DeepCopy(const biorbd::rigidbody::BoneMesh& other);

    // Related to the mesh points

    ///
    /// \brief Add a point to the mesh
    /// \param node The point to add
    ///
    void addPoint(const biorbd::utils::Node3d & node);

    ///
    /// \brief Return a point at a specific position
    /// \param i Position
    /// \return A point at position i
    ///
    const biorbd::utils::Node3d& point(unsigned int i) const;

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
    unsigned int sizePatch();

    
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
    std::shared_ptr<std::vector<biorbd::utils::Node3d>> m_mesh; ///< The bone mesh
    std::shared_ptr<std::vector<biorbd::rigidbody::Patch>> m_patch; ///< The patch
    std::shared_ptr<biorbd::utils::Path> m_pathFile; ///< The path to the mesh file
};

}}

#endif // BIORBD_RIGIDBODY_BONE_MESH_H
