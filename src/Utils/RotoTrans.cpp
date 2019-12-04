#define BIORBD_API_EXPORTS
#include "Utils/RotoTrans.h"

#include <rbdl/rbdl_math.h>
#include "Utils/Error.h"
#include "Utils/Vector3d.h"
#include "Utils/String.h"
#include "Utils/Vector.h"
#include "Utils/Rotation.h"

biorbd::utils::RotoTrans::RotoTrans(
        const Eigen::Matrix4d& matrix) :
    Eigen::Matrix4d(matrix)
{
}

biorbd::utils::RotoTrans::RotoTrans(
        const biorbd::utils::Rotation& rot) :
    Eigen::Matrix4d(combineRotAndTrans(rot, Eigen::Vector3d::Zero()))
{
#ifndef SKIP_ASSERT
    biorbd::utils::Error::check(this->squaredNorm() == 1.,
                                "The RotoTrans norm is not equal to one");
#endif
}

biorbd::utils::RotoTrans::RotoTrans(
        const biorbd::utils::Rotation& rot,
        const biorbd::utils::Vector3d& trans) :
    Eigen::Matrix4d(combineRotAndTrans(rot,trans))
{
#ifndef SKIP_ASSERT
    biorbd::utils::Error::check(this->squaredNorm() == 1.,
                                "The RotoTrans norm is not equal to one");
#endif
}

biorbd::utils::RotoTrans::RotoTrans(
        const biorbd::utils::Vector& rotation,
        const biorbd::utils::Vector3d& translation,
        const biorbd::utils::String& rotationSequence) :
    Eigen::Matrix4d(fromEulerAngles(rotation, translation, rotationSequence))
{
#ifndef SKIP_ASSERT
    biorbd::utils::Error::check(this->squaredNorm() == 1.,
                                "The RotoTrans norm is not equal to one");
#endif
}

biorbd::utils::RotoTrans::RotoTrans(const RigidBodyDynamics::Math::SpatialTransform& st) :
    Eigen::Matrix4d(fromSpatialTransform(st))
{
#ifndef SKIP_ASSERT
    biorbd::utils::Error::check(this->squaredNorm() == 1.,
                                "The RotoTrans norm is not equal to one");
#endif
}

biorbd::utils::Vector3d biorbd::utils::RotoTrans::axe(int idx)
{
    biorbd::utils::Error::check(
                idx>=0 && idx<=2, "Axis must be between 0 and 2 included");
    return rot().block(0,idx,3,1);
}

biorbd::utils::RotoTrans biorbd::utils::RotoTrans::transpose() const
{
    biorbd::utils::RotoTrans tp;
    tp.block(0, 0, 3, 3) = block(0, 0, 3, 3).transpose();
    tp.block(0, 3, 3, 1) = -tp.block(0, 0, 3, 3) * block(0, 3, 3, 1);
    tp.block(3, 0, 1, 4) << 0, 0, 0, 1;
    return tp;
}

biorbd::utils::Vector3d biorbd::utils::RotoTrans::trans() const
{
    return this->block(0,3,3,1);
}

biorbd::utils::Rotation biorbd::utils::RotoTrans::rot() const
{
    return this->block(0,0,3,3);
}

biorbd::utils::RotoTrans& biorbd::utils::RotoTrans::combineRotAndTrans(
        const biorbd::utils::Rotation& rot,
        const biorbd::utils::Vector3d& trans){
    block(0,0,3,3) = rot;
    block(0,3,3,1) = trans;
    block(3,0,1,4) << 0,0,0,1;
    return *this;
}

biorbd::utils::RotoTrans& biorbd::utils::RotoTrans::fromSpatialTransform(
        const RigidBodyDynamics::Math::SpatialTransform& st)
{
    return combineRotAndTrans(st.E,st.r);
}

biorbd::utils::RotoTrans& biorbd::utils::RotoTrans::fromEulerAngles(
        const biorbd::utils::Vector& rot,
        const biorbd::utils::Vector3d& trans,
        const biorbd::utils::String& seq)
{
    biorbd::utils::Rotation rot_mat;
    rot_mat.fromEulerAngles(rot, seq);
    block(0,0,3,3) = rot_mat;
    block(0,3,3,1) = trans;
    block(3,0,1,4) << 0,0,0,1;
    return *this;
}

biorbd::utils::Vector biorbd::utils::RotoTrans::toEulerAngles(
        const biorbd::utils::RotoTrans& rt,
        const biorbd::utils::String &seq)
{
    return biorbd::utils::Rotation::toEulerAngles(rt.block(0, 0, 3, 3), seq);
}

biorbd::utils::RotoTrans biorbd::utils::RotoTrans::mean(const std::vector<RotoTrans> & mToMean)
{
    // The translation part is just the actual mean
    Eigen::Vector3d v_tp;
    v_tp.setZero();
    for (unsigned int i = 0; i<mToMean.size(); ++i){
        v_tp += mToMean[i].trans();
    }
    v_tp = v_tp/mToMean.size();

    // The rotation part should call the proper way implemented in Rotation
    std::vector<biorbd::utils::Rotation> rotations;
    for (unsigned int i=0; i<mToMean.size(); ++i){
        rotations.push_back(mToMean[i].block(0, 0, 3, 3));
    }
    biorbd::utils::RotoTrans m_out(
                biorbd::utils::Rotation::mean(rotations), v_tp);
    return m_out;
}

Eigen::Vector4d biorbd::utils::RotoTrans::expand3dTo4d(const biorbd::utils::Vector3d &v1)
{
    Eigen::Vector4d v2;
    v2.block(0,0,3,1) = v1;
    v2(3) = 1;
    return v2;
}

void biorbd::utils::RotoTrans::checkUnitary()
{
#ifndef SKIP_ASSERT
    biorbd::utils::Error::check(this->rot().squaredNorm() == 1.,
                                "The RotoTrans norm is not equal to one");
    biorbd::utils::Error::check(this->block(3, 0, 1, 4).sum() == 1.,
                                "Last row of the RotoTrans should be (0,0,0,1");
    biorbd::utils::Error::check((*this)(3, 3) == 1.,
                                "Last row of the RotoTrans should be (0,0,0,1");
#endif
}

std::ostream &operator<<(std::ostream &os, const biorbd::utils::RotoTrans &a)
{
    os << a.block(0,0,4,4);
    return os;
}
