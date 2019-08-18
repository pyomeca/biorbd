#ifndef BIORBD_RIGIDBODY_CONTACTS_H
#define BIORBD_RIGIDBODY_CONTACTS_H

#include <vector>
#include <rbdl/Constraints.h>
#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd { namespace utils {
class Attitude;
class Node;
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
    virtual ~Contacts();
    unsigned int AddConstraint(
            unsigned int body_id,
            const biorbd::utils::Node &body_point,
            const biorbd::utils::Node &world_normal,
            const biorbd::utils::String& name,
            double acc = 0);
    unsigned int AddConstraint(
            unsigned int body_id,
            const biorbd::utils::Node &body_point,
            const biorbd::utils::String& axis,
            const biorbd::utils::String& name,
            double acc = 0);
    virtual unsigned int AddLoopConstraint(
            unsigned int body_id_predecessor,
            unsigned int body_id_successor,
            const biorbd::utils::Attitude& X_predecessor,
            const biorbd::utils::Attitude& X_successor,
            const biorbd::utils::Vector& axis,
            bool enableStabilization = false,
            double stabilizationParam = 0.1,
            const biorbd::utils::String& name = biorbd::utils::String() );

    Contacts &getConstraints_nonConst(const biorbd::rigidbody::Joints& jointsModel); // La premiere fois il faut appeler cette fonction avec cet arguement, ensuite, il n'est plus utile
    Contacts &getConstraints_nonConst();
    const Contacts &getConstraints() const;

    bool hasContacts() const;
    unsigned int nContacts() const;

    biorbd::utils::String name(unsigned int i);
    std::vector<biorbd::utils::Node> constraintsInGlobal(
            biorbd::rigidbody::Joints& m,
            const biorbd::rigidbody::GeneralizedCoordinates &Q,
            bool updateKin); // Retourne la position des contraintes dans le rep`ere global

    // Set and get
    biorbd::utils::Vector getForce() const;
protected:
    unsigned int m_nbreConstraint;
    bool m_binded;

};

}}

#endif // BIORBD_RIGIDBODY_CONTACTS_H
