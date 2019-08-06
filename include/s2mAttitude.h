#ifndef S2MATTITUDE_H
#define S2MATTITUDE_H

    #include "biorbdConfig.h"
    #include "s2mError.h"
    #include "s2mString.h"
    #include <Eigen/Dense>
    #include "rbdl/rbdl.h"
    #include <iostream>
class s2mNode;
class BIORBD_API s2mAttitude : public Eigen::Matrix4d
{
    public:
        s2mAttitude(const Eigen::Matrix4d& = Eigen::Matrix4d::Identity());
        s2mAttitude(const Eigen::VectorXd&, const Eigen::Vector3d&, const s2mString&);
        s2mAttitude(const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans = Eigen::Vector3d::Zero());
        s2mAttitude(const RigidBodyDynamics::Math::SpatialTransform&);
        virtual ~s2mAttitude();

        Eigen::Vector3d axe(int); // Aller récupérer un axe en particulier

        s2mAttitude transpose() const;
        Eigen::Vector3d trans() const;
        Eigen::Matrix3d rot() const;
        void setIdentity();
        bool isIdentity();

        static s2mAttitude SpatialTransform2Attitude(const RigidBodyDynamics::Math::SpatialTransform& st);
        static s2mAttitude combineRotAndTrans(const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans);
        static Eigen::Matrix4d transformCardanToMatrix(const Eigen::VectorXd&, const Eigen::Vector3d&, const s2mString&);
        static Eigen::VectorXd transformMatrixToCardan(const s2mAttitude&, const s2mString &seq);

        const s2mAttitude operator*(const s2mAttitude&);
        const Eigen::Vector3d operator*(const Eigen::Vector3d&);
        const Eigen::Vector3d operator*(const Eigen::Vector4d&);
        const s2mNode operator*(const s2mNode&);

        static s2mAttitude mean(const std::vector<s2mAttitude>&); // Moyenne des matrices 4x4
    protected:
        Eigen::Vector4d expand3dTo4d(const Eigen::Vector3d&);
    private:
};
std::ostream& operator<<(std::ostream& os, const s2mAttitude &a);

#include "s2mNode.h"

#endif // S2MATTITUDE_H
