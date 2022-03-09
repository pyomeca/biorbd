#ifndef BIORBD_RIGIDBODY_CONTACTS_H
#define BIORBD_RIGIDBODY_CONTACTS_H

#include <vector>
#include <memory>
#include <rbdl/Constraints.h>
#include "biorbdConfig.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
class RotoTrans;
class Vector3d;
class Vector;
class String;
class SpatialVector;
}

namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;
class GeneralizedAcceleration;
class NodeSegment;

///
/// \brief Class Contacts
///
#ifdef SWIG
class BIORBD_API Contacts
#else
class BIORBD_API Contacts : public RigidBodyDynamics::ConstraintSet
#endif
{
public:
    ///
    /// \brief Construct contacts
    ///
    Contacts();

    ///
    /// \brief Deep copy of contacts
    /// \return Copy of contacts
    ///
    Contacts DeepCopy() const;

    ///
    /// \brief Deep copy of contacts
    /// \param other The contacts to copy
    ///
    void DeepCopy(
        const Contacts& other);

    ///
    /// \brief Add a constraint to the constraint set
    /// \param body_id The body which is affected directly by the constraint
    /// \param body_point The point that is constrained relative to the contact body
    /// \param world_normal The normal along the constraint acts (in base coordinates)
    /// \param name A human readable name
    ///
    unsigned int AddConstraint(
        unsigned int body_id,
        const utils::Vector3d &body_point,
        const utils::Vector3d &world_normal,
        const utils::String& name);

    ///
    /// \brief Add a constraint to the constraint set
    /// \param body_id The body which is affected directly by the constraint
    /// \param body_point The point that is constrained relative to the contact body
    /// \param axis The axis along which the constraint acts
    /// \param name A human readable name
    ///

    unsigned int AddConstraint(
        unsigned int body_id,
        const utils::Vector3d &body_point,
        const utils::String& axis,
        const utils::String& name);

    ///
    /// \brief Add a loop constraint to the constraint set
    /// \param body_id_predecessor The identifier of the predecessor body
    /// \param body_id_successor The identifier of the successor body
    /// \param X_predecessor A spatial transform localizing the constrained frames on the predecessor body, expressed with respect to the predecessor body frame
    /// \param X_successor A spatial transform localizing the constrained frames on the successor body, expressed with respect to the successor body frame
    /// \param axis A spatial vector indicating the axis along which the constraint acts
    /// \param name A human readable name
    /// \param enableStabilization Whether stabilization should be enabled or not
    /// \param stabilizationParam The value used for stabilization
    ///
    unsigned int AddLoopConstraint(
        unsigned int body_id_predecessor,
        unsigned int body_id_successor,
        const utils::RotoTrans& X_predecessor,
        const utils::RotoTrans& X_successor,
        const utils::SpatialVector& axis,
        const utils::String& name,
        bool enableStabilization = false,
        double stabilizationParam = 0.1);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~Contacts();

    ///
    /// \brief Get constraints
    /// \return The constraints
    ///
    Contacts &getConstraints();

    ///
    /// \brief Check if there are contacts
    /// \return The presence of contacts
    ///
    bool hasContacts() const;

    ///
    /// \brief Return the number of contacts
    /// \return The number of contacts
    ///
    unsigned int nbContacts() const;

    ///
    /// \brief Return the name of the all contacts
    /// \return The name of the contacts
    ///
    std::vector<utils::String> contactNames();

    ///
    /// \brief Return the name of the contact of a specified axis
    /// \param i The axis
    /// \return The name of the contact of a specified axis
    ///
    utils::String contactName(unsigned int i);

    ///
    /// \brief Return a vector with the sorted axis index considered in the rigid contact
    /// \param contact_idx the index of the rigid contact
    /// \return A vector with the sorted axis index considered in the rigid contact
    ///
    std::vector<int> rigidContactAxisIdx(unsigned int contact_idx) const;

    ///
    /// \brief Return the constraints position in the global reference
    /// \param Q The generalized coordinates of the joints
    /// \param updateKin Whether the kinematics of the model should be updated from Q
    /// \return The constraints positions in the global reference
    ///
    std::vector<utils::Vector3d> constraintsInGlobal(
        const GeneralizedCoordinates &Q,
        bool updateKin);

    ///
    /// \brief Return the force acting on the constraint
    /// \return The force acting on the constraint
    ///
    utils::Vector getForce() const;

    ///
    /// \brief Return the segment idx of the contact in biorbd formalism
    /// \param idx The index of the contact
    /// \return segment idx of the contact in biorbd formalism
    ///
    int contactSegmentBiorbdId(
            int  idx) const;

    ///
    /// \brief Return the index of rigid contacts for a specified segment index
    /// \param idx The index of the segment
    /// \return the index of rigid contacts for the specified segment idx
    ///
    std::vector<size_t> segmentRigidContactIdx(
            int segment_idx) const;

    ///
    /// \brief Get the rigid contacts in a list of spatial vector of dimension 6xNdof
    /// \param Q The Generalized coordinates
    /// \param f_contacts the forces applied at the contact points
    /// \return The rigid contacts
    ///
    std::vector<RigidBodyDynamics::Math::SpatialVector>* rigidContactToSpatialVector(
            const GeneralizedCoordinates& Q,
            std::vector<utils::Vector> *f_contacts,
            bool updateKin);

    ///
    /// \brief Get the rigid contacts in a list of spatial vector of dimension 6xNdof
    /// \param applicationPoint the position where the force is applied in base coordinate
    /// \param sorted_axis_index vector of the sorted axis index considered in the rigid contact
    /// \param f_contact the forces applied at the contact point
    /// \return The rigid contacts
    ///
    utils::SpatialVector computeForceAtOrigin(
            utils::Vector3d applicationPoint,
            std::vector<int> sortedAxisIndex,
            utils::Vector f_contact);

    ///
    /// \brief Returns the number of rigid contacts (ignoring the loop constraints)
    /// \return The number of rigid contacts (ignoring the loop constraints)
    ///
    int nbRigidContacts() const;

    ///
    /// \brief Returns all the rigid contacts as declared in the model
    /// \return All the rigid contacts as declared in the model
    ///
    const std::vector<NodeSegment>& rigidContacts() const;

    ///
    /// \brief Returns the rigid contact idx as declared in the model
    /// \param idx The index of the contact
    /// \return All the rigid contacts as declared in the model
    ///
    const NodeSegment& rigidContact(unsigned int idx) const;

    ///
    /// \brief Return the rigidContact position in the global reference
    /// \param Q The generalized coordinates of the joints
    /// \param idx The index of the contact
    /// \param updateKin Whether the kinematics of the model should be updated from Q
    /// \return The rigidContact position in the global reference
    ///
    utils::Vector3d rigidContact(
        const GeneralizedCoordinates &Q,
        unsigned int idx,
        bool updateKin);

    ///
    /// \brief Return all the rigidContacts position in the global reference
    /// \param Q The generalized coordinates of the joints
    /// \param updateKin Whether the kinematics of the model should be updated from Q
    /// \return All the rigidContacts positions in the global reference
    ///
    std::vector<utils::Vector3d> rigidContacts(
        const GeneralizedCoordinates &Q,
        bool updateKin);

    ///
    /// \brief Return the velocity of the chosen contact
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param idx The index of the contact
    /// \param updateKin If the model should be updated
    /// \return The velocity of the chosen contact
    ///
    utils::Vector3d rigidContactVelocity(
        const rigidbody::GeneralizedCoordinates &Q,
        const rigidbody::GeneralizedVelocity &Qdot,
        unsigned int idx,
        bool updateKin = true);

    ///
    /// \brief Return the velocities of all the contacts
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized velocities
    /// \return The velocities of all the contacts
    ///
    std::vector<utils::Vector3d> rigidContactsVelocity(
        const rigidbody::GeneralizedCoordinates &Q,
        const rigidbody::GeneralizedVelocity &Qdot,
        bool updateKin = true);

    ///
    /// \brief Return the acceleration of the chosen contact
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized velocities
    /// \param idx The index of the contact
    /// \param updateKin If the model should be updated
    /// \return The acceleration of the chosen contact
    ///
    utils::Vector3d rigidContactAcceleration(
        const rigidbody::GeneralizedCoordinates &Q,
        const rigidbody::GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &dQdot,
        unsigned int idx,
        bool updateKin = true);

    ///
    /// \brief Return the acceleration of all the contacts
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized velocities
    /// \return The acceleration of all the contacts
    ///
    std::vector<utils::Vector3d> rigidContactsAcceleration(
        const rigidbody::GeneralizedCoordinates &Q,
        const rigidbody::GeneralizedVelocity &Qdot,
        const rigidbody::GeneralizedAcceleration &dQdot,
        bool updateKin = true);

protected:
    std::shared_ptr<unsigned int> m_nbreConstraint; ///< Number of constraints
    std::shared_ptr<bool> m_isBinded; ///< If the model is ready
    std::shared_ptr<std::vector<rigidbody::NodeSegment>> m_rigidContacts; ///< The rigid contacts declared in the model (copy of RBDL information)

};

}
}

#endif // BIORBD_RIGIDBODY_CONTACTS_H
