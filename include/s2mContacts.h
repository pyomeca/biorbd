#ifndef S2M_CONTACTS_H
#define S2M_CONTACTS_H

#include <vector>
#include <rbdl/Constraints.h>
#include "biorbdConfig.h"
#include "Utils/String.h"

class s2mJoints;
namespace biorbd { namespace utils {
class Attitude;
class Node;
class Vector;
class GenCoord;
class String;
}}
class BIORBD_API s2mContacts : public RigidBodyDynamics::ConstraintSet
{
public:
    s2mContacts();
    virtual ~s2mContacts();
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
            const double stabilizationParam = 0.1,
            const biorbd::utils::String& name = biorbd::utils::String() );

    s2mContacts &getConstraints_nonConst(const s2mJoints& jointsModel); // La premiere fois il faut appeler cette fonction avec cet arguement, ensuite, il n'est plus utile
    s2mContacts &getConstraints_nonConst();
    const s2mContacts &getConstraints() const;

    bool hasContacts() const;
    unsigned int nContacts() const;

    biorbd::utils::String name(unsigned int i);
    std::vector<biorbd::utils::Node> constraintsInGlobal(
            s2mJoints& m,
            const biorbd::utils::GenCoord &Q,
            const bool updateKin); // Retourne la position des contraintes dans le rep`ere global

    // Set and get
    biorbd::utils::Vector getForce() const;
protected:
    unsigned int m_nbreConstraint;
    bool m_binded;

};

#endif // S2M_CONTACTS_H
