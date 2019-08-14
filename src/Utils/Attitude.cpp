#define BIORBD_API_EXPORTS
#include "Utils/Attitude.h"

#include <iostream>
#include <rbdl/rbdl_math.h>
#include "Utils/Error.h"
#include "Utils/String.h"
#include "Utils/Node.h"

biorbd::utils::Attitude::Attitude(const Eigen::Matrix4d& m) :
    Eigen::Matrix4d(m)
{
}

biorbd::utils::Attitude::Attitude(
        const Eigen::Matrix3d& rot,
        const Eigen::Vector3d& trans) :
    Eigen::Matrix4d(combineRotAndTrans(rot,trans))
{

}
biorbd::utils::Attitude::Attitude(
        const Eigen::VectorXd& rot,
        const Eigen::Vector3d& trans,
        const biorbd::utils::String& seq) :
    Eigen::Matrix4d(transformCardanToMatrix(rot, trans,seq))
{

}
biorbd::utils::Attitude::Attitude(const RigidBodyDynamics::Math::SpatialTransform& st) :
    Eigen::Matrix4d(SpatialTransform2Attitude(st))
{

}

Eigen::Vector3d biorbd::utils::Attitude::axe(int i)
{
    biorbd::utils::Error::error(i>=0 && i<=2, "Axis must be between 0 and 2 included");
    return rot().block(0,i,3,1);
}

void biorbd::utils::Attitude::setIdentity(){
    this->block(0,0,4,4) << 1,0,0,0,
                            0,1,0,0,
                            0,0,1,0,
            0,0,0,1;
}

bool biorbd::utils::Attitude::isIdentity()
{
    Attitude eye;
    eye.setIdentity();
    return *this == eye;
}

biorbd::utils::Attitude biorbd::utils::Attitude::SpatialTransform2Attitude(const RigidBodyDynamics::Math::SpatialTransform& st){
    return combineRotAndTrans(st.E.transpose(),st.r);
}

biorbd::utils::Attitude biorbd::utils::Attitude::combineRotAndTrans(
        const Eigen::Matrix3d& rot,
        const Eigen::Vector3d& trans){
    Attitude tp;
    tp.block(0,0,3,3) = rot;
    tp.block(0,3,3,1) = trans;
    tp.block(3,0,1,4) << 0,0,0,1;
    return tp;
}

biorbd::utils::Attitude biorbd::utils::Attitude::transpose() const{
    Attitude tp;
    tp.block(0,0,3,3) = this->block(0,0,3,3).transpose();
    tp.block(0,3,3,1) =-tp.block(0,0,3,3) * this->block(0,3,3,1);
    tp.block(3,0,1,4) << 0,0,0,1;
    return tp;
}

Eigen::Vector3d biorbd::utils::Attitude::trans() const
{
    return this->block(0,3,3,1);
}

Eigen::Matrix3d biorbd::utils::Attitude::rot() const
{
    return this->block(0,0,3,3);
}


Eigen::Matrix4d biorbd::utils::Attitude::transformCardanToMatrix(
        const Eigen::VectorXd& v,
        const Eigen::Vector3d& t,
        const biorbd::utils::String& seq){
    // S'assurer que le vecteur et la sequence d'angle aient le mpeme nombre d'élément
    biorbd::utils::Error::error(seq.length() == static_cast<unsigned int>(v.rows()), "Rotation and sequence of rotation must be the same length");

    Eigen::Matrix4d tp1;
    tp1.setIdentity();
    // Trouver la matrice de rotation
    for (unsigned int i=0; i<seq.length(); ++i){
        Eigen::Matrix3d tp;
        if (seq.tolower()[i] == 'x')
            // Matrice de rotation en x
            tp <<   1,          0,          0,
                    0,          std::cos(v[i]),  -std::sin(v[i]),
                    0,          std::sin(v(i)),  std::cos(v(i));

        else if (seq.tolower()[i] == 'y')
            // Matrice de rotation en y
            tp <<   std::cos(v(i)),  0,          std::sin(v(i)),
                    0,          1,          0,
                    -std::sin(v(i)), 0,          std::cos(v(i));

        else if (seq.tolower()[i] == 'z')
            // Matrice de rotation en z
            tp <<   std::cos(v(i)),  -std::sin(v(i)), 0,
                    std::sin(v(i)),  std::cos(v(i)),  0,
                    0,          0,          1;
        else
            biorbd::utils::Error::error(0, "Rotation sequence not recognized");

        tp1.block(0,0,3,3) = tp1.block(0,0,3,3) * tp;
    }
    tp1.block(0,3,3,1) = t;
    return tp1;
}

Eigen::VectorXd biorbd::utils::Attitude::transformMatrixToCardan(
        const Attitude& a,
        const biorbd::utils::String &seq) {
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
        biorbd::utils::Error::error(false, "Angle sequence is not recognized");
    }

    return v;
}


Eigen::Vector4d biorbd::utils::Attitude::expand3dTo4d(const Eigen::Vector3d &v1)
{
    Eigen::Vector4d v2;
    v2.block(0,0,3,1) = v1;
    v2(3) = 1;
    return v2;
}


const biorbd::utils::Attitude biorbd::utils::Attitude::operator*(const Attitude& s1){
    Attitude out;
    out.block(0,0,4,4) = static_cast<Eigen::Matrix4d>(*this) * static_cast<Eigen::Matrix4d>(s1);
    return out;
}
const biorbd::utils::Node biorbd::utils::Attitude::operator*(const biorbd::utils::Node &n)
{
    biorbd::utils::Node out(n);
    Eigen::Vector4d v(static_cast<Eigen::Matrix4d>(*this) * expand3dTo4d(out.position()));
    out.setPosition(v);
    return out;
}

biorbd::utils::Attitude biorbd::utils::Attitude::mean(const std::vector<Attitude> & mToMean)
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
    Attitude m_out(svd.matrixU() * svd.matrixV().transpose(), v_tp);
    return m_out;
}

const Eigen::Vector3d biorbd::utils::Attitude::operator*(const Eigen::Vector4d &v)
{
    Eigen::Vector4d out (static_cast<Eigen::Matrix4d>(*this) * static_cast<Eigen::Vector4d>(v));
    return out.block(0,0,3,1);
}
const Eigen::Vector3d biorbd::utils::Attitude::operator*(const Eigen::Vector3d &v)
{
    return operator*(expand3dTo4d(v));
}

std::ostream& operator<<(std::ostream& os, const biorbd::utils::Attitude &a){
    os << a.block(0,0,4,4);
    return os;
}
