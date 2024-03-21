#define BIORBD_API_EXPORTS
#include "Utils/Rotation.h"

#include "Utils/Error.h"
#include "Utils/Vector3d.h"
#include "Utils/SpatialTransform.h"
#include "Utils/String.h"
#include "Utils/Vector.h"
#include "RigidBody/NodeSegment.h"

using namespace BIORBD_NAMESPACE;

utils::Rotation::Rotation(
    const utils::Matrix3d& matrix) :
    utils::Matrix3d(matrix)
{
    checkUnitary();
}

#ifdef BIORBD_USE_CASADI_MATH

utils::Rotation::Rotation(
    const RBDLCasadiMath::MX_Xd_static<3, 3> &other) :
    utils::Matrix3d(other)
{

}

utils::Rotation::Rotation(
    const RBDLCasadiMath::MX_Xd_SubMatrix& other) :
    utils::Matrix3d(other)
{

}
#endif

utils::Rotation::Rotation(
    const utils::Scalar& v00, const utils::Scalar& v01, const utils::Scalar& v02,
    const utils::Scalar& v10, const utils::Scalar& v11, const utils::Scalar& v12,
    const utils::Scalar& v20, const utils::Scalar& v21, const utils::Scalar& v22) :
    utils::Matrix3d (v00, v01, v02, v10, v11, v12, v20, v21, v22)
{

}

utils::Rotation::Rotation(
    const utils::Vector& rotation,
    const utils::String& rotationSequence) :
    utils::Matrix3d(fromEulerAngles(rotation, rotationSequence))
{

}

utils::Rotation::Rotation(
    const utils::SpatialTransform &st) :
    utils::Matrix3d(st.E)
{

}

utils::Vector3d utils::Rotation::axe(size_t idx) const
{
    utils::Error::check(idx<=2, "Axis must be between 0 and 2 included");
    return this->block(0, static_cast<unsigned int>(idx), 3, 1);
}

utils::Rotation utils::Rotation::fromSpatialTransform(
    const utils::SpatialTransform& st)
{
    return st.E;
}

utils::Rotation utils::Rotation::fromEulerAngles(
    const utils::Vector &rot,
    const utils::String& seq)
{
    // Check for size consistency
    utils::Error::check(
        seq.length() == static_cast<size_t>(rot.rows()),
        "Rotation and sequence of rotation must be the same length");

    utils::Rotation out;
    out.setIdentity();
    // Set the actual rotation matrix to this
    utils::Matrix3d tp;
    for (size_t i=0; i<seq.length(); ++i) {
        utils::Scalar cosVi(std::cos(rot[static_cast<unsigned int>(i)]));
        utils::Scalar sinVi(std::sin(rot[static_cast<unsigned int>(i)]));
        if (seq.tolower()[i] == 'x')
            tp = utils::Matrix3d(1,     0,      0,
                                 0, cosVi, -sinVi,
                                 0, sinVi,  cosVi);

        else if (seq.tolower()[i] == 'y')
            tp = utils::Matrix3d(cosVi, 0, sinVi,
                                 0, 1,     0,
                                 -sinVi, 0, cosVi);

        else if (seq.tolower()[i] == 'z')
            tp = utils::Matrix3d(cosVi, -sinVi,  0,
                                 sinVi,  cosVi,  0,
                                 0,      0,  1);
        else {
            utils::Error::raise("Rotation sequence not recognized");
        }

        out.block(0,0,3,3) = out.block(0,0,3,3) * tp;
    }
    return out;
}

utils::Rotation utils::Rotation::fromMarkersNonNormalized(
    const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment> &axis1markers,
    const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment> &axis2markers,
    const std::pair<utils::String, utils::String>& axesNames,
    const utils::String &axisToRecalculate)
{
    if (!axesNames.first.compare("") || !axesNames.second.compare("")) {
        utils::Error::raise("axesNames must be defined with a pair of \"x\", \"y\" or \"z\"");
    }

    // Figure out where to put the axes
    std::vector<size_t> map(3);
    std::vector<size_t> toMultiply(2);
    if (!axesNames.first.tolower().compare("x")) {
        map[0] = 0;
        if (!axesNames.second.tolower().compare("y")) {
            map[1] = 1;
            map[2] = 2;
            toMultiply[0] = 0;
            toMultiply[1] = 1;
        } else {
            map[1] = 2;
            map[2] = 1;
            toMultiply[0] = 2;
            toMultiply[1] = 0;
        }
    } else if (!axesNames.first.tolower().compare("y")) {
        map[0] = 1;
        if (!axesNames.second.tolower().compare("x")) {
            map[1] = 0;
            map[2] = 2;
            toMultiply[0] = 0;
            toMultiply[1] = 1;
        } else {
            map[1] = 2;
            map[2] = 0;
            toMultiply[0] = 1;
            toMultiply[1] = 2;
        }
    } else if (!axesNames.first.tolower().compare("z")) {
        map[0] = 2;
        if (!axesNames.second.tolower().compare("x")) {
            map[1] = 0;
            map[2] = 1;
            toMultiply[0] = 2;
            toMultiply[1] = 0;
        } else {
            map[1] = 1;
            map[2] = 0;
            toMultiply[0] = 1;
            toMultiply[1] = 2;
        }
    }

    // Get the system of axis XYZ
    std::vector<utils::Vector3d> axes(3);
    axes[map[0]] = axis1markers.second - axis1markers.first;
    axes[map[1]] = axis2markers.second - axis2markers.first;
    axes[map[2]] = axes[toMultiply[0]].cross(axes[toMultiply[1]]);

    // Recalculate one axis
    if (!axisToRecalculate.tolower().compare("x")) {
        axes[0] = axes[1].cross(axes[2]);
    } else if (!axisToRecalculate.tolower().compare("y")) {
        axes[1] = axes[2].cross(axes[0]);
    } else if (!axisToRecalculate.tolower().compare("z")) {
        axes[2] = axes[0].cross(axes[1]);
    }

    // Organize them in a non-normalized matrix
    utils::Rotation r_out;
    for (size_t i=0; i<3; ++i) {
        r_out.block(0, static_cast<unsigned int>(i), 3, 1) = axes[i];
    }
    return r_out;
}

utils::Rotation utils::Rotation::fromMarkers(
    const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment> &axis1markers,
    const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment> &axis2markers,
    const std::pair<utils::String, utils::String>& axesNames,
    const utils::String &axisToRecalculate)
{
    utils::Rotation r_out(
        fromMarkersNonNormalized(axis1markers, axis2markers, axesNames,
                                 axisToRecalculate));

    // Organize them in a normalized matrix
    for (size_t i=0; i<3; ++i) {
        // Normalize axes
        r_out.block<3, 1>(0, static_cast<unsigned int>(i)).normalize();
    }

    return r_out;
}

utils::Vector utils::Rotation::toEulerAngles(
    const utils::Rotation &r,
    const utils::String &seq)
{
    utils::Vector v;
    if (!seq.compare("zyzz")) {
        v = utils::Vector(3);
    } else {
        v = utils::Vector(seq.length());
    }

    // sequences taken from https://pdfslide.net/documents/euler-angles-58fcebd3620f7.html
    if (!seq.compare("x")) {
        v[0] = std::asin(r(2, 1));           // x
    } else if (!seq.compare("y")) {
        v[0] = std::asin(r(0, 2));           // y
    } else if (!seq.compare("z")) {
        v[0] = std::asin(r(1, 0));           // z
    } else if (!seq.compare("xy")) {
        v[0] = std::asin(r(2,1));            // x
        v[1] = std::asin(r(0,2));            // y
    } else if (!seq.compare("xz")) {
        v[0] = -std::asin(r(1,2));           // x
        v[1] = -std::asin(r(0,1));           // z
    } else if (!seq.compare("yx")) {
        v[0] = -std::asin(r(2,0));           // y
        v[1] = -std::asin(r(1,2));           // x
    } else if (!seq.compare("yz")) {
        v[0] = std::asin(r(0,2));            // y
        v[1] = std::asin(r(1,0));            // z
    } else if (!seq.compare("zx")) {
        v[0] = std::asin(r(1,0));            // z
        v[1] = std::asin(r(2,1));            // x
    } else if (!seq.compare("zy")) {
        v[0] = -std::asin(r(0,1));           // z
        v[1] = -std::asin(r(2,0));           // y
    } else if (!seq.compare("xyz")) {
        v[0] = std::atan2(-r(1,2), r(2,2));  // x
        v[1] = std::asin(r(0,2));            // y
        v[2] = std::atan2(-r(0,1), r(0,0));  // z
    } else if (!seq.compare("xzy")) {
        v[0] = std::atan2(r(2,1), r(1,1));   // x
        v[1] = std::asin(-r(0,1));           // z
        v[2] = std::atan2(r(0,2), r(0,0));   // y
    } else if (!seq.compare("yxy")) {
        v[0] = std::atan2(r(0,1), r(2,1));   // y
        v[1] = std::acos(r(1,1));            // x
        v[2] = std::atan2(r(1,0), -r(1,2));  // y
    } else if (!seq.compare("yxz")) {
        v[0] = std::atan2(r(0,2), r(2,2));   // y
        v[1] = std::asin(-r(1,2));           // x
        v[2] = std::atan2(r(1,0), r(1,1));   // z
    } else if (!seq.compare("yzx")) {
        v[0] = std::atan2(-r(2,0), r(0,0));  // y
        v[1] = std::asin(r(1,0));            // z
        v[2] = std::atan2(-r(1,2), r(1,1));  // x
    } else if (!seq.compare("yzy")) {
	v[0] = std::atan2(r(2, 1), -r(0, 1));// y
        v[1] = std::acos(r(1, 1));           // z
        v[2] = std::atan2(r(1, 2), r(1, 0)); // y
    } else if (!seq.compare("zxy")) {
        v[0] = std::atan2(-r(0,1), r(1,1));  // z
        v[1] = std::asin(r(2,1));            // x
        v[2] = std::atan2(-r(2,0), r(2,2));  // y
    } else if (!seq.compare("zyx")) {
        v[0] = std::atan2(r(1,0), r(0,0));   // z
        v[1] = std::asin(-r(2,0));           // y
        v[2] = std::atan2(r(2,1), r(2,2));   // x
    } else if (!seq.compare("zxz")) {
        v[0] = std::atan2(r(0,2), -r(1,2));  // z
        v[1] = std::acos(r(2,2));            // x
        v[2] = std::atan2(r(2,0), r(2,1));   // z
    } else if (!seq.compare("zyz")) {
        v[0] = std::atan2(r(1,2), r(0,2));   // z
        v[1] = std::acos(r(2,2));            // y
        v[2] = std::atan2(r(2,1), -r(2,0));  // z
    } else if (!seq.compare("zyzz")) {
        v[0] = std::atan2(r(1,2), r(0,2));   // z
        v[1] = std::acos(r(2,2));            // y
        v[2] = std::atan2(r(2,1), -r(2,0)) + v[0];   // z+z
    } else {
        utils::Error::raise("Angle sequence is not recognized");
    }

    return v;
}

#ifndef BIORBD_USE_CASADI_MATH
utils::Rotation utils::Rotation::mean(
    const std::vector<utils::Rotation> & mToMean)
{
    utils::Matrix3d m_tp;
    m_tp.setZero();

    // Initial guess being the actual arithmetic mean
    for (size_t i = 0; i<mToMean.size(); ++i) {
        m_tp += mToMean[i];
    }
    m_tp = m_tp/mToMean.size();

    // SVD decomposition
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        m_tp, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Normalize the matrix
    utils::Rotation m_out(svd.matrixU() * svd.matrixV().transpose());

    return m_out;
}
#endif

void utils::Rotation::checkUnitary()
{
#ifndef BIORBD_USE_CASADI_MATH
#ifndef SKIP_ASSERT
    double sqrtNorm = static_cast<double>(this->squaredNorm());
    bool checkUnit(fabs(sqrtNorm - 3.) < 1e-4);
    if (!checkUnit)
    {
        utils::Matrix3d R(orthoNormalize());
        utils::Error::raise(utils::String("The Rotation matrix square norm is equal to ")
                        + sqrtNorm + ", but should be equal to 3. Consider replacing the RT matrix by this closest orthonormal matrix \n[" 
                        + R(0, 0) + ", " + R(0, 1) + ", " + R(0, 2) + "\n" 
                        + R(1, 0) + ", " + R(1, 1) + ", " + R(1, 2) + "\n" 
                        + R(2, 0) + ", " + R(2, 1) + ", " + R(2, 2) + "]\n" 
                        + "Alternatively, you can compile with SKIP_ASSERT set to ON to turn off this error message");

    }

#endif
#endif
}

std::ostream &operator<<(std::ostream &os, const utils::Rotation &a)
{
    os << a.block(0,0,3,3);
    return os;
}

#ifdef BIORBD_USE_CASADI_MATH
void utils::Rotation::operator=(
    const RBDLCasadiMath::MX_Xd_static<3, 3> &other)
{
    this->Matrix3d::operator=(other);
}

void utils::Rotation::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix& other)
{
    this->Matrix3d::operator=(other);
}
#endif
