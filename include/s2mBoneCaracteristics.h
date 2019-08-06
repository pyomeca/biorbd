#ifndef S2MBONECARACTERISTICS_H
#define S2MBONECARACTERISTICS_H
    #include "biorbdConfig.h"
    #include <Eigen/Dense>
    #include "s2mBoneMesh.h"
    #include <rbdl/rbdl.h>

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
    private:
//        double m_mass;
        double m_length;
        s2mBoneMesh m_mesh;
//        Eigen::Vector3d m_centerOfMass;
//        Eigen::Matrix3d m_matrixOfInertia;
};

#endif // S2MBONECARACTERISTICS_H
