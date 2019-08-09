#ifndef S2MCONTACTS_H
#define S2MCONTACTS_H

#include <vector>
#include <rbdl/Constraints.h>
#include "biorbdConfig.h"

class s2mNode;
class s2mVector;
class s2mJoints;
class s2mString;
class s2mGenCoord;
class BIORBD_API s2mContacts : public RigidBodyDynamics::ConstraintSet
{
    public:
        s2mContacts();
        virtual ~s2mContacts();
        unsigned int AddConstraint(unsigned int body_id,
                                           const s2mNode &body_point,
                                           const s2mNode &world_normal,
                                           const s2mString& name,
                                           double acc = 0);
        unsigned int AddConstraint(unsigned int body_id,
                                           const s2mNode &body_point,
                                           const s2mString& axis,
                                           const s2mString& name,
                                           double acc = 0);
        virtual unsigned int AddLoopConstraint(
                unsigned int body_id_predecessor,
                unsigned int body_id_successor,
                const s2mAttitude& X_predecessor,
                const s2mAttitude& X_successor,
                const s2mVector& axis,
                bool enableStabilization = false,
                const double stabilizationParam = 0.1,
                const s2mString& name = s2mString() );

        const s2mContacts &getConstraints(const s2mJoints& jointsModel); // La premiere fois il faut appeler cette fonction avec cet arguement, ensuite, il n'est plus utile
        const s2mContacts &getConstraints();

        bool hasContacts() const;
        unsigned int nContacts() const;

        s2mString name(unsigned int i);
        std::vector<s2mNode> constraintsInGlobal(s2mJoints& m,
                                                 const s2mGenCoord &Q,
                                                 const bool updateKin); // Retourne la position des contraintes dans le rep`ere global

        // Set and get
        s2mVector getForce() const;
    protected:
        unsigned int m_nbreConstraint;
        bool m_binded;

};

#endif // S2MCONTACTS_H
