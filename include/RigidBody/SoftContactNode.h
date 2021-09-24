#ifndef BIORBD_RIGIDBODY_SOFT_CONTACT_NODE_H
#define BIORBD_RIGIDBODY_SOFT_CONTACT_NODE_H

#include "biorbdConfig.h"
#include "RigidBody/NodeSegment.h"
#include "rbdl/rbdl_math.h"

namespace BIORBD_NAMESPACE
{

namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;

///
/// \brief A point attached to a contact node, generally speaking a skin marker
///
class BIORBD_API SoftContactNode : public rigidbody::NodeSegment
{
public:
    ///
    /// \brief Construct a contact node
    ///
    SoftContactNode();

    ///
    /// \brief Construct a contact node
    /// \param x X-Component of the node
    /// \param y Y-Component of the node
    /// \param z Z-Component of the node
    ///
    SoftContactNode(
        const utils::Scalar& x,
        const utils::Scalar& y,
        const utils::Scalar& z);

    ///
    /// \brief Construct a contact node from another node
    /// \param other The other node
    ///
    SoftContactNode(
        const utils::Vector3d& other);

    ///
    /// \brief Construct a contact node
    /// \param x X-Component of the node
    /// \param y Y-Component of the node
    /// \param z Z-Component of the node
    /// \param name The name of the node
    /// \param parentName The name of the parent
    /// \param parentID The index of the parent contact
    ///
    SoftContactNode(
        const utils::Scalar& x,
        const utils::Scalar& y,
        const utils::Scalar& z,
        const utils::String& name,
        const utils::String& parentName,
        int parentID);

    ///
    /// \brief Construct a contact node
    /// \param node The position of the node
    /// \param name The name of the node
    /// \param parentName The name of the parent
    /// \param parentID The index of the parent contact
    ///
    SoftContactNode(
        const utils::Vector3d& node,
        const utils::String& name,
        const utils::String& parentName,
        int parentID);

    ///
    /// \brief Deep copy of the contact node
    /// \param other The contact node to copy
    ///
    void DeepCopy(const SoftContactNode& other);

    ///
    /// \brief Get the force in a spatial vector
    /// \param Q The Generalized Coordinates
    /// \param QDot The Generalized velocities
    /// \return The Spatial vector
    ///
    virtual RigidBodyDynamics::Math::SpatialVector computeForce(
            const GeneralizedCoordinates& Q,
            const GeneralizedVelocity& QDot) const = 0;

protected:
    ///
    /// \brief Set the type of the contact node
    ///
    void setType();

};

}
}

#endif // BIORBD_RIGIDBODY_SOFT_CONTACT_NODE_H
