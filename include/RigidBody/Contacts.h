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
    Contacts();
    biorbd::rigidbody::Contacts DeepCopy() const;
    void DeepCopy(const biorbd::rigidbody::Contacts& other);

    unsigned int AddConstraint(
            unsigned int body_id,
            const biorbd::utils::Node3d &body_point,
            const biorbd::utils::Node3d &world_normal,
            const biorbd::utils::String& name,
            double acc = 0);
    unsigned int AddConstraint(
            unsigned int body_id,
            const biorbd::utils::Node3d &body_point,
            const biorbd::utils::String& axis,
            const biorbd::utils::String& name,
            double acc = 0);
    unsigned int AddLoopConstraint(
            unsigned int body_id_predecessor,
            unsigned int body_id_successor,
            const biorbd::utils::RotoTrans& X_predecessor,
            const biorbd::utils::RotoTrans& X_successor,
            const biorbd::utils::Vector& axis,
            const biorbd::utils::String& name,
            bool enableStabilization = false,
            double stabilizationParam = 0.1);
    virtual ~Contacts();

    Contacts &getConstraints_nonConst();
    const Contacts &getConstraints();

    bool hasContacts() const;
    unsigned int nContacts() const;

    biorbd::utils::String name(unsigned int i);
    std::vector<biorbd::utils::Node3d> constraintsInGlobal(
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin); // Retourne la position des contraintes dans le rep`ere global

    // Set and get
    biorbd::utils::Vector getForce() const;
protected:
    std::shared_ptr<unsigned int> m_nbreConstraint;
    std::shared_ptr<bool> m_isBinded;

};

}}

#endif // BIORBD_RIGIDBODY_CONTACTS_H
