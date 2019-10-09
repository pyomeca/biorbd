#define BIORBD_API_EXPORTS
#include "Utils/RotoTrans.h"

#include <rbdl/rbdl_math.h>
#include "Utils/Error.h"
#include "Utils/Node3d.h"
#include "Utils/String.h"

biorbd::utils::RotoTrans::RotoTrans(const Eigen::Matrix4d& m) :
    Eigen::Matrix4d(m)
{

}

biorbd::utils::RotoTrans::RotoTrans(
        const Eigen::Matrix3d& rot,
        const Eigen::Vector3d& trans) :
    Eigen::Matrix4d(combineRotAndTrans(rot,trans))
{

}
biorbd::utils::RotoTrans::RotoTrans(const Eigen::VectorXd& rotation,
        const Eigen::Vector3d& translation,
        const biorbd::utils::String& rotationSequence) :
    Eigen::Matrix4d(transformCardanToMatrix(rotation, translation, rotationSequence))
{

}
biorbd::utils::RotoTrans::RotoTrans(const RigidBodyDynamics::Math::SpatialTransform& st) :
    Eigen::Matrix4d(SpatialTransform2RotoTrans(st))
{

}

Eigen::Vector3d biorbd::utils::RotoTrans::axe(int i)
{
    biorbd::utils::Error::check(i>=0 && i<=2, "Axis must be between 0 and 2 included");
    return rot().block(0,i,3,1);
}

biorbd::utils::RotoTrans& biorbd::utils::RotoTrans::SpatialTransform2RotoTrans(const RigidBodyDynamics::Math::SpatialTransform& st)
{
    return combineRotAndTrans(st.E,st.r);
}

biorbd::utils::RotoTrans& biorbd::utils::RotoTrans::combineRotAndTrans(
        const Eigen::Matrix3d& rot,
        const Eigen::Vector3d& trans){
    block(0,0,3,3) = rot;
    block(0,3,3,1) = trans;
    block(3,0,1,4) << 0,0,0,1;
    return *this;
}

biorbd::utils::RotoTrans biorbd::utils::RotoTrans::transpose() const
{
    biorbd::utils::RotoTrans tp;
    tp.block(0,0,3,3) = block(0,0,3,3).transpose();
    tp.block(0,3,3,1) = block(0,0,3,3) * block(0,3,3,1);
    tp.block(3,0,1,4) << 0,0,0,1;
    return tp;
}

Eigen::Vector3d biorbd::utils::RotoTrans::trans() const
{
    return this->block(0,3,3,1);
}

Eigen::Matrix3d biorbd::utils::RotoTrans::rot() const
{
    return this->block(0,0,3,3);
}


biorbd::utils::RotoTrans& biorbd::utils::RotoTrans::transformCardanToMatrix(
        const Eigen::VectorXd& rot,
        const Eigen::Vector3d& trans,
        const biorbd::utils::String& seq)
{
    // S'assurer que le vecteur et la sequence d'angle aient le mpeme nombre d'élément
    biorbd::utils::Error::check(seq.length() == static_cast<unsigned int>(rot.rows()), "Rotation and sequence of rotation must be the same length");

    setIdentity();
    // Trouver la matrice de rotation
    Eigen::Matrix3d tp;
    for (unsigned int i=0; i<seq.length(); ++i){
        double cosVi(std::cos(rot[i]));
        double sinVi(std::sin(rot[i]));
        if (seq.tolower()[i] == 'x')
            // Matrice de rotation en x
            tp <<   1,     0,      0,
                    0, cosVi, -sinVi,
                    0, sinVi,  cosVi;

        else if (seq.tolower()[i] == 'y')
            // Matrice de rotation en y
            tp <<   cosVi, 0, sinVi,
                        0, 1,     0,
                   -sinVi, 0, cosVi;

        else if (seq.tolower()[i] == 'z')
            // Matrice de rotation en z
            tp <<  cosVi, -sinVi,  0,
                   sinVi,  cosVi,  0,
                       0,      0,  1;
        else
            biorbd::utils::Error::raise("Rotation sequence not recognized");

        block(0,0,3,3) *= tp;
    }
    block(0,3,3,1) = trans;
    return *this;
}

Eigen::VectorXd biorbd::utils::RotoTrans::transformMatrixToCardan(
        const RotoTrans& a,
        const biorbd::utils::String &seq)
{
    Eigen::VectorXd v;
    if (!seq.compare("zyzz"))
        v = Eigen::VectorXd(3);
    else
        v = Eigen::VectorXd(seq.length());


    if (!seq.compare("x")) {
        v[0] = asin(a(2, 1));           // x
    } else if (!seq.compare("y")) {
        v[0] = asin(a(0, 2));           // y
    } else if (!seq.compare("z")) {
        v[0] = asin(a(1, 0));           // z
    } else if (!seq.compare("xy")) {
        v[0] = asin(a(2,1));            // x
        v[1] = asin(a(0,2));            // y
    } else if (!seq.compare("xz")) {
        v[0] = -asin(a(1,2));           // x
        v[1] = -asin(a(0,1));           // z
    } else if (!seq.compare("yx")) {
        v[0] = -asin(a(2,0));           // y
        v[1] = -asin(a(1,2));           // x
    } else if (!seq.compare("yz")) {
        v[0] = asin(a(0,2));            // y
        v[1] = asin(a(1,0));            // z
    } else if (!seq.compare("zx")) {
        v[0] = asin(a(1,0));            // z
        v[1] = asin(a(2,1));            // x
    } else if (!seq.compare("zy")) {
        v[0] = -asin(a(0,1));           // z
        v[1] = -asin(a(2,0));           // y
    } else if (!seq.compare("xyz")) {
        v[0] = atan2(-a(1,2), a(2,2));  // x
        v[1] = asin(a(0,2));            // y
        v[2] = atan2(-a(0,1), a(0,0));  // z
    } else if (!seq.compare("xzy")) {
        v[0] = atan2(a(2,1), a(1,1));   // x
        v[1] = asin(-a(0,1));           // z
        v[2] = atan2(a(0,2), a(0,0));   // y
    } else if (!seq.compare("yxz")) {
        v[0] = atan2(a(0,2), a(2,2));   // y
        v[1] = asin(-a(1,2));           // x
        v[2] = atan2(a(1,0), a(1,1));   // z
    } else if (!seq.compare("yzx")) {
        v[0] = atan2(-a(2,0), a(0,0));  // y
        v[1] = asin(a(1,0));            // z
        v[2] = atan2(-a(1,2), a(1,1));  // x
    } else if (!seq.compare("zxy")) {
        v[0] = atan2(-a(0,1), a(1,1));  // z
        v[1] = asin(a(2,1));            // x
        v[2] = atan2(-a(2,0), a(2,2));  // y
    } else if (!seq.compare("zyx")) {
        v[0] = atan2(a(1,0), a(0,0));   // z
        v[1] = asin(-a(2,0));           // y
        v[2] = atan2(a(2,1), a(2,2));   // x
    } else if (!seq.compare("zxz")) {
        v[0] = atan2(a(0,2), -a(1,2));  // z
        v[1] = acos(a(2,2));            // x
        v[2] = atan2(a(2,0), a(2,1));   // z
    } else if (!seq.compare("zyz")) {
        v[0] = atan2(a(1,2), a(0,2));   // z
        v[1] = acos(a(2,2));            // y
        v[2] = atan2(a(2,1), -a(2,0));  // z
    } else if (!seq.compare("zyzz")) {
        v[0] = atan2(a(1,2), a(0,2));   // z
        v[1] = acos(a(2,2));            // y
        v[2] = atan2(a(2,1), -a(2,0)) + v[0];   // z+z
    }

    else {
        biorbd::utils::Error::raise("Angle sequence is not recognized");
    }

    return v;
}

Eigen::Vector4d biorbd::utils::RotoTrans::expand3dTo4d(const Eigen::Vector3d &v1)
{
    Eigen::Vector4d v2;
    v2.block(0,0,3,1) = v1;
    v2(3) = 1;
    return v2;
}

biorbd::utils::RotoTrans biorbd::utils::RotoTrans::mean(const std::vector<RotoTrans> & mToMean)
{
    Eigen::Matrix3d m_tp; // matrice rot tp
    Eigen::Vector3d v_tp; // translation tp
    m_tp.setZero();
    v_tp.setZero();

    // Commencer par la moyenne arithmétique éléments par éléments
    for (unsigned int i = 0; i<mToMean.size(); ++i){
        m_tp += mToMean[i].rot();
        v_tp += mToMean[i].trans();
    }
    m_tp = m_tp/mToMean.size();
    v_tp = v_tp/mToMean.size();

    // Faire la décomposition svd
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(m_tp, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Normaliser la matrice
    RotoTrans m_out(svd.matrixU() * svd.matrixV().transpose(), v_tp);
    return m_out;
}

std::ostream &operator<<(std::ostream &os, const biorbd::utils::RotoTrans &a)
{
    os << a.block(0,0,4,4);
    return os;
}
