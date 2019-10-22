#ifndef BIORBD_RIGIDBODY_CONTACTS_H
#define BIORBD_RIGIDBODY_CONTACTS_H

#include <vector>
#include <memory>
#include <rbdl/Constraints.h>
#include "biorbdConfig.h"

namespace biorbd { namespace utils {
class RotoTrans;
class Node3d;
class Vector;
class String;
}}

namespace biorbd {
namespace rigidbody {
class Joints;
class GeneralizedCoordinates;

class BIORBD_API Contacts : public RigidBodyDynamics::ConstraintSet
{
public:
    ///
    /// \brief Create contacts
    ///
    Contacts();

    /// 
    /// \brief Deep copy of contacts
    /// \return Copy of contacts
    ///
    biorbd::rigidbody::Contacts DeepCopy() const;

    ///
    /// \brief Deep copy of contacts 
    /// \param other TODO: ?
    /// 
    void DeepCopy(const biorbd::rigidbody::Contacts& other);

    ///
    /// \brief Add a constraint
    /// \param body_id The ID of the body on which to add constraint
    /// \param body_point The point of the body on which to add constraint
    /// \param world_normal TODO: ?
    /// \param name The name of the constraint
    /// \param acc Equals 0 by default
    ///
    unsigned int AddConstraint(
            unsigned int body_id,
            const biorbd::utils::Node3d &body_point,
            const biorbd::utils::Node3d &world_normal,
            const biorbd::utils::String& name,
            double acc = 0);

    ///
    /// \brief Add a constraint
    /// \param body_id The ID of the body on which to add constraint
    /// \param axis The axis
    /// \param name The name of the constraint
    /// \param acc Equals 0 by default
    ///

    unsigned int AddConstraint(
            unsigned int body_id,
            const biorbd::utils::Node3d &body_point,
            const biorbd::utils::String& axis,
            const biorbd::utils::String& name,
            double acc = 0);

    ///
    /// \brief Add a loop constraint
    /// \param body_id_predecessor The body ID of the predecessor
    /// \param body_id_successor The body ID of the successor
    /// \param X_predecessor The predecessor
    /// \param X_successor The successor
    /// \param axis The axis 
    /// \param name The name of the constraint
    /// \param enableStabilization False by default
    /// \param stabilizationParam equals 0.1 by default
    ///
    unsigned int AddLoopConstraint(
            unsigned int body_id_predecessor,
            unsigned int body_id_successor,
            const biorbd::utils::RotoTrans& X_predecessor,
            const biorbd::utils::RotoTrans& X_successor,
            const biorbd::utils::Vector& axis,
            const biorbd::utils::String& name,
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
    /// \return True or False
    ///
    bool hasContacts() const;

    ///
    /// \brief Return the number of contacts
    /// \return The number of contacts
    ///
    unsigned int nbContacts() const;

    ///
    /// \brief Return the name of the contact at a specified position
    /// \param i The position
    /// \return The name of the contact at position i
    ///
    biorbd::utils::String name(unsigned int i);

    ///
    /// \brief Return the contraints position in the global reference
    /// \param Q Generalized coordinates
    /// \param updateKin Update kinematics
    /// \return The contraints positions in the global reference
    ///
    std::vector<biorbd::utils::Node3d> constraintsInGlobal(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin); 

    // Set and get
    /// 
    /// \brief Return the force 
    /// \return The force
    ///
    biorbd::utils::Vector getForce() const;
protected:
    std::shared_ptr<unsigned int> m_nbreConstraint; ///< Number of constraints
    std::shared_ptr<bool> m_isBinded; ///< TODO:? Binded: true or false

};

}}

#endif // BIORBD_RIGIDBODY_CONTACTS_H
