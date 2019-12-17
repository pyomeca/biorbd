#define BIORBD_API_EXPORTS
#include "Utils/Rotation.h"

#include <rbdl/rbdl_math.h>
#include "Utils/Error.h"
#include "Utils/Vector3d.h"
#include "Utils/String.h"
#include "Utils/Vector.h"

biorbd::utils::Rotation::Rotation(
        const Eigen::Matrix3d& matrix) :
    Eigen::Matrix3d(matrix)
{
    checkUnitary();
}

biorbd::utils::Rotation::Rotation(
        const biorbd::utils::Vector& rotation,
        const biorbd::utils::String& rotationSequence) :
    Eigen::Matrix3d(fromEulerAngles(rotation, rotationSequence))
{

}

biorbd::utils::Rotation::Rotation(
        const RigidBodyDynamics::Math::SpatialTransform &st) :
    Eigen::Matrix3d(st.E)
{

}

biorbd::utils::Vector biorbd::utils::Rotation::axe(int idx)
{
    biorbd::utils::Error::check(
                idx>=0 && idx<=2, "Axis must be between 0 and 2 included");
    return this->block(0,idx,3,1);
}

biorbd::utils::Rotation biorbd::utils::Rotation::fromSpatialTransform(
        const RigidBodyDynamics::Math::SpatialTransform& st)
{
    return st.E;
}

biorbd::utils::Rotation& biorbd::utils::Rotation::fromEulerAngles(
        const Eigen::VectorXd& rot,
        const biorbd::utils::String& seq)
{
    // Check for size consistency
    biorbd::utils::Error::check(
                seq.length() == static_cast<unsigned int>(rot.rows()),
                "Rotation and sequence of rotation must be the same length");

    setIdentity();
    // Set the actual rotation matrix to this
    Eigen::Matrix3d tp;
    for (unsigned int i=0; i<seq.length(); ++i){
        double cosVi(std::cos(rot[i]));
        double sinVi(std::sin(rot[i]));
        if (seq.tolower()[i] == 'x')
            tp <<   1,     0,      0,
                    0, cosVi, -sinVi,
                    0, sinVi,  cosVi;

        else if (seq.tolower()[i] == 'y')
            tp <<   cosVi, 0, sinVi,
                        0, 1,     0,
                   -sinVi, 0, cosVi;

        else if (seq.tolower()[i] == 'z')
            tp <<  cosVi, -sinVi,  0,
                   sinVi,  cosVi,  0,
                       0,      0,  1;
        else
            biorbd::utils::Error::raise("Rotation sequence not recognized");

        block(0,0,3,3) *= tp;
    }
    return *this;
}

biorbd::utils::Vector biorbd::utils::Rotation::toEulerAngles(
        const biorbd::utils::Rotation &rt,
        const biorbd::utils::String &seq)
{
    biorbd::utils::Vector v;
    if (!seq.compare("zyzz"))
        v = biorbd::utils::Vector(3);
    else
        v = biorbd::utils::Vector(static_cast<unsigned int>(seq.length()));

    if (!seq.compare("x")) {
        v[0] = asin(rt(2, 1));           // x
    }
    else if (!seq.compare("y")) {
        v[0] = asin(rt(0, 2));           // y
    }
    else if (!seq.compare("z")) {
        v[0] = asin(rt(1, 0));           // z
    }
    else if (!seq.compare("xy")) {
        v[0] = asin(rt(2,1));            // x
        v[1] = asin(rt(0,2));            // y
    }
    else if (!seq.compare("xz")) {
        v[0] = -asin(rt(1,2));           // x
        v[1] = -asin(rt(0,1));           // z
    }
    else if (!seq.compare("yx")) {
        v[0] = -asin(rt(2,0));           // y
        v[1] = -asin(rt(1,2));           // x
    }
    else if (!seq.compare("yz")) {
        v[0] = asin(rt(0,2));            // y
        v[1] = asin(rt(1,0));            // z
    }
    else if (!seq.compare("zx")) {
        v[0] = asin(rt(1,0));            // z
        v[1] = asin(rt(2,1));            // x
    }
    else if (!seq.compare("zy")) {
        v[0] = -asin(rt(0,1));           // z
        v[1] = -asin(rt(2,0));           // y
    }
    else if (!seq.compare("xyz")) {
        v[0] = atan2(-rt(1,2), rt(2,2));  // x
        v[1] = asin(rt(0,2));            // y
        v[2] = atan2(-rt(0,1), rt(0,0));  // z
    }
    else if (!seq.compare("xzy")) {
        v[0] = atan2(rt(2,1), rt(1,1));   // x
        v[1] = asin(-rt(0,1));           // z
        v[2] = atan2(rt(0,2), rt(0,0));   // y
    }
    else if (!seq.compare("yxz")) {
        v[0] = atan2(rt(0,2), rt(2,2));   // y
        v[1] = asin(-rt(1,2));           // x
        v[2] = atan2(rt(1,0), rt(1,1));   // z
    }
    else if (!seq.compare("yzx")) {
        v[0] = atan2(-rt(2,0), rt(0,0));  // y
        v[1] = asin(rt(1,0));            // z
        v[2] = atan2(-rt(1,2), rt(1,1));  // x
    }
    else if (!seq.compare("zxy")) {
        v[0] = atan2(-rt(0,1), rt(1,1));  // z
        v[1] = asin(rt(2,1));            // x
        v[2] = atan2(-rt(2,0), rt(2,2));  // y
    }
    else if (!seq.compare("zyx")) {
        v[0] = atan2(rt(1,0), rt(0,0));   // z
        v[1] = asin(-rt(2,0));           // y
        v[2] = atan2(rt(2,1), rt(2,2));   // x
    }
    else if (!seq.compare("zxz")) {
        v[0] = atan2(rt(0,2), -rt(1,2));  // z
        v[1] = acos(rt(2,2));            // x
        v[2] = atan2(rt(2,0), rt(2,1));   // z
    }
    else if (!seq.compare("zyz")) {
        v[0] = atan2(rt(1,2), rt(0,2));   // z
        v[1] = acos(rt(2,2));            // y
        v[2] = atan2(rt(2,1), -rt(2,0));  // z
    }
    else if (!seq.compare("zyzz")) {
        v[0] = atan2(rt(1,2), rt(0,2));   // z
        v[1] = acos(rt(2,2));            // y
        v[2] = atan2(rt(2,1), -rt(2,0)) + v[0];   // z+z
    }
    else {
        biorbd::utils::Error::raise("Angle sequence is not recognized");
    }

    return v;
}

biorbd::utils::Rotation biorbd::utils::Rotation::mean(
        const std::vector<biorbd::utils::Rotation> & mToMean)
{
    Eigen::Matrix3d m_tp;
    m_tp.setZero();

    // Initial guess being the actual arithmetic mean
    for (unsigned int i = 0; i<mToMean.size(); ++i){
        m_tp += mToMean[i];
    }
    m_tp = m_tp/mToMean.size();

    // SVD decomposition
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
                m_tp, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Normalize the matrix
    biorbd::utils::Rotation m_out(svd.matrixU() * svd.matrixV().transpose());

    return m_out;
}

void biorbd::utils::Rotation::checkUnitary()
{
#ifndef SKIP_ASSERT
    biorbd::utils::Error::check(fabs(this->squaredNorm() - 3.) < 1e-10,
                                "The Rotation matrix norm is not equal to one");
#endif
}

std::ostream &operator<<(std::ostream &os, const biorbd::utils::Rotation &a)
{
    os << a.block(0,0,3,3);
    return os;
}
