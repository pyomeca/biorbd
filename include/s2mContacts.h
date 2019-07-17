#ifndef S2MCONTACTS_H
#define S2MCONTACTS_H

#include "biorbdConfig.h"
#include "s2mString.h"
#include "rbdl/rbdl.h"
#include "s2mJoints.h"
#include "s2mError.h"
#include "s2mGenCoord.h"


class BIORBD_API s2mContacts : public RigidBodyDynamics::ConstraintSet
{
    public:
        s2mContacts();
        virtual unsigned int AddConstraint(unsigned int body_id,
                                           const s2mNode &body_point,
                                           const s2mNode &world_normal,
                                           const s2mString& name,
                                           double acc = 0);
        virtual unsigned int AddConstraint(unsigned int body_id,
                                           const s2mNode &body_point,
                                           const s2mString& axis,
                                           const s2mString& name,
                                           double acc = 0);

        s2mContacts &getConstraints(const RigidBodyDynamics::Model& jointsModel); // La premiere fois il faut appeler cette fonction avec cet arguement, ensuite, il n'est plus utile
        s2mContacts &getConstraints();

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
    private:
};

#endif // S2MCONTACTS_H
