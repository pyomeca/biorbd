#ifndef S2M_BONE_CARACTERISTICS_H
#define S2M_BONE_CARACTERISTICS_H

#include <rbdl/Body.h>
#include "biorbdConfig.h"
#include "s2mBoneMesh.h"

class BIORBD_API s2mBoneCaracteristics : public RigidBodyDynamics::Body
{
    public:
        s2mBoneCaracteristics();
        s2mBoneCaracteristics(const double &mass, // Mass of the body
                              const s2mNode &com, // Center of Mass
                              const RigidBodyDynamics::Math::Matrix3d &inertia, // Inertia matrix
                              const s2mBoneMesh &mesh = s2mBoneMesh()) ; // position des meshings de l'os
        virtual ~s2mBoneCaracteristics();

        // Set and Get
        double length() const;
        double mass() const;
        void setLength(const double &val);
        const s2mBoneMesh& mesh() const;
        const Eigen::Matrix3d& inertia() const;

    protected:
        double m_length;
        s2mBoneMesh m_mesh;
};

#endif // S2M_BONE_CARACTERISTICS_H
