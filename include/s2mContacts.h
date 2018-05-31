#ifndef S2MCONTACTS_H
#define S2MCONTACTS_H

#include "biorbdConfig.h"
#include "s2mString.h"
#include "rbdl/rbdl.h"
#include "s2mJoints.h"
#include "s2mError.h"
#include "s2mGenCoord.h"


class BIORBD_API s2mContacts : private RigidBodyDynamics::ConstraintSet
{
    public:
        s2mContacts();
        virtual ~s2mContacts();
        virtual unsigned int AddConstraint(unsigned int, const RigidBodyDynamics::Math::Vector3d&, const RigidBodyDynamics::Math::Vector3d&, const s2mString&, double = 0);
        virtual unsigned int AddConstraint(unsigned int, const RigidBodyDynamics::Math::Vector3d&, const s2mString&, const s2mString&, double = 0);

        RigidBodyDynamics::ConstraintSet& getConstraints(const RigidBodyDynamics::Model&); // La premiere fois il faut appeler cette fonction avec cet arguement, ensuite, il n'est plus utile
        RigidBodyDynamics::ConstraintSet& getConstraints();

        bool hasContacts() const {if (m_nbreConstraint>0) return true; else return false;}
        unsigned int nContacts() const { return m_nbreConstraint;}

        s2mString name(unsigned int i) {s2mError::s2mAssert(i<m_nbreConstraint, "Idx for name is too high.."); return RigidBodyDynamics::ConstraintSet::name[i];}
        std::vector<Eigen::Vector3d> constraintsInGlobal(s2mJoints&, const s2mGenCoord&, const bool updateKin = true); // Retourne la position des contraintes dans le rep`ere global

        // Set and get
    protected:
        unsigned int m_nbreConstraint;
        bool m_binded;
    private:
};

#endif // S2MCONTACTS_H
