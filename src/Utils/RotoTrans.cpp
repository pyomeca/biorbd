#define BIORBD_API_EXPORTS
#include "Utils/RotoTrans.h"

#include "Utils/Error.h"
#include "Utils/Vector3d.h"
#include "Utils/String.h"
#include "Utils/Vector.h"
#include "Utils/Rotation.h"
#include "Utils/SpatialTransform.h"

#include "RigidBody/NodeSegment.h"

using namespace BIORBD_NAMESPACE;

utils::RotoTrans::RotoTrans(
    const RigidBodyDynamics::Math::Matrix4d& matrix) :
    RigidBodyDynamics::Math::Matrix4d(matrix)
{
    checkUnitary();
}

utils::RotoTrans::RotoTrans(
    const utils::Scalar& v00, const utils::Scalar& v01,
    const utils::Scalar& v02,
    const utils::Scalar& v03,
    const utils::Scalar& v10, const utils::Scalar& v11,
    const utils::Scalar& v12,
    const utils::Scalar& v13,
    const utils::Scalar& v20, const utils::Scalar& v21,
    const utils::Scalar& v22,
    const utils::Scalar& v23,
    const utils::Scalar& v30, const utils::Scalar& v31,
    const utils::Scalar& v32,
    const utils::Scalar& v33) :
    RigidBodyDynamics::Math::Matrix4d (v00, v01, v02, v03,
                                       v10, v11, v12, v13,
                                       v20, v21, v22, v23,
                                       v30, v31, v32, v33)
{
    checkUnitary();
}

utils::RotoTrans::RotoTrans(
    const utils::Rotation& rot) :
    RigidBodyDynamics::Math::Matrix4d(combineRotAndTrans(rot, utils::Vector3d::Zero()))
{

}

utils::RotoTrans::RotoTrans(
    const utils::Rotation& rot,
    const utils::Vector3d& trans) :
    RigidBodyDynamics::Math::Matrix4d(combineRotAndTrans(rot,trans))
{

}

utils::RotoTrans::RotoTrans(
    const utils::Vector& rotation,
    const utils::Vector3d& translation,
    const utils::String& rotationSequence) :
    RigidBodyDynamics::Math::Matrix4d(
        fromEulerAngles(rotation, translation, rotationSequence))
{

}

utils::RotoTrans::RotoTrans(
    const utils::SpatialTransform& st) :
    RigidBodyDynamics::Math::Matrix4d(fromSpatialTransform(st))
{

}

utils::RotoTrans utils::RotoTrans::fromMarkers(
    const rigidbody::NodeSegment& origin,
    const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment> &axis1markers,
    const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment> &axis2markers,
    const std::pair<utils::String, utils::String>& axesNames,
    const utils::String &axisToRecalculate)
{
    RotoTrans rt_out;
    rt_out.block(0, 0, 3, 3) = 
        Rotation::fromMarkers(axis1markers, axis2markers, axesNames, axisToRecalculate);
    rt_out.block(0, 3, 3, 1) = origin;
    return rt_out;
}

utils::Vector3d utils::RotoTrans::axe(size_t idx) const
{
    utils::Error::check(
        idx<=2, "Axis must be between 0 and 2 included");
    return rot().block(0, static_cast<unsigned int>(idx),3,1);
}

utils::RotoTrans utils::RotoTrans::transpose() const
{
    utils::RotoTrans tp;
    tp.block(0, 0, 3, 3) = block(0, 0, 3, 3).transpose();
    tp.block(0, 3, 3, 1) = 
        -tp.block(0, 0, 3, 3) * block(0, 3, 3, 1);
    tp.block(3, 0, 1, 4) = RigidBodyDynamics::Math::Vector4d(0, 0, 0, 1).transpose();
    return tp;
}

utils::Vector3d utils::RotoTrans::trans() const
{
    return this->block<3, 1>(0,3);
}

utils::Rotation utils::RotoTrans::rot() const
{
    return this->block<3, 3>(0,0);
}

utils::RotoTrans utils::RotoTrans::combineRotAndTrans(
    const utils::Rotation& rot,
    const utils::Vector3d& trans)
{
    utils::RotoTrans out;
    out.block(0,0,3,3) = rot;
    out.block(0,3,3,1) = trans;
    out.block(3, 0, 1, 4) = RigidBodyDynamics::Math::Vector4d(0, 0, 0, 1).transpose();
    return out;
}

utils::RotoTrans utils::RotoTrans::fromSpatialTransform(
    const utils::SpatialTransform& st)
{
    return combineRotAndTrans(st.rotation(), st.translation());
}

utils::RotoTrans utils::RotoTrans::fromEulerAngles(
    const utils::Vector& rot,
    const utils::Vector3d& trans,
    const utils::String& seq)
{

    utils::Rotation rot_mat(utils::Rotation::fromEulerAngles(rot, seq));

    utils::RotoTrans out;
    out.block(0, 0, 3, 3) = rot_mat;
    out.block(0, 3, 3, 1) = trans;
    return out;
}

utils::Vector utils::RotoTrans::toEulerAngles(
    const utils::RotoTrans& rt,
    const utils::String &seq)
{
    return utils::Rotation::toEulerAngles(rt.block<3, 3>(0, 0), seq);
}

#ifndef BIORBD_USE_CASADI_MATH
utils::RotoTrans utils::RotoTrans::mean(
    const std::vector<RotoTrans> & mToMean)
{
    // The translation part is just the actual mean
    RigidBodyDynamics::Math::Vector3d v_tp;
    v_tp.setZero();
    for (size_t i = 0; i<mToMean.size(); ++i) {
        v_tp += mToMean[i].trans();
    }
    v_tp = v_tp/mToMean.size();

    // The rotation part should call the proper way implemented in Rotation
    std::vector<utils::Rotation> rotations;
    for (size_t i=0; i<mToMean.size(); ++i) {
        rotations.push_back(mToMean[i].block<3, 3>(0, 0));
    }
    utils::RotoTrans m_out(
        utils::Rotation::mean(rotations), v_tp);
    return m_out;
}
#endif

RigidBodyDynamics::Math::Vector4d utils::RotoTrans::expand3dTo4d(
    const utils::Vector3d &v1)
{
    RigidBodyDynamics::Math::Vector4d v2;
    v2.block(0,0,3,1) = v1;
    v2(3) = 1;
    return v2;
}

void utils::RotoTrans::checkUnitary()
{
#ifndef BIORBD_USE_CASADI_MATH
#ifndef SKIP_ASSERT
    this->rot(); // Automatically cast the test for the rotation part
    utils::Error::check(this->block(3, 0, 1, 4).sum() == 1., "Last row of the RotoTrans should be (0,0,0,1");
    utils::Error::check((*this)(3, 3) == 1., "Last row of the RotoTrans should be (0,0,0,1");
#endif
#endif
}

std::ostream &operator<<(std::ostream &os, const utils::RotoTrans &a)
{
    os << a.block(0,0,4,4);
    return os;
}
