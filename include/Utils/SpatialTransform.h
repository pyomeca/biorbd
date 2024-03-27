#ifndef BIORBD_UTILS_SPATIAL_TRANSFORM_H
#define BIORBD_UTILS_SPATIAL_TRANSFORM_H

#include "biorbdConfig.h"
#include "rbdl/rbdl_math.h"
#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE
{
namespace utils
{
    class Vector3d;
    class Matrix3d;
    class RotoTrans;

///
/// \brief Wrapper of the SpatialTransform (rotation and translation)
///
#ifdef SWIG
class BIORBD_API SpatialTransform
#else
class BIORBD_API SpatialTransform : public RigidBodyDynamics::Math::SpatialTransform
#endif
{
public:
    ///
    /// \brief Construct SpatialTransform
    ///
    SpatialTransform();

    ///
    /// \brief Construct SpatialTransform from other SpatialTransform
    /// \param other The SpatialTransform to copy
    ///
    SpatialTransform(
        const SpatialTransform& other);

    ///
    /// \brief Construct SpatialTransform from rotation and translation
    /// \param translation The translation part of the transformation
    /// \param rotation The rotation part of the transformation
    ///
    SpatialTransform(
        const Matrix3d& rotation,
        const Vector3d& translation);

    ///
    /// \brief Construct SpatialTransform from a rototranslation matrix
    /// \param rototrans The transformation matrix to construct from
    ///
    SpatialTransform(
        const RotoTrans& rototrans);

#ifndef SWIG
    ///
    /// \brief Construct SpatialTransform from other SpatialTransform
    /// \param other The SpatialTransform to copy
    ///
    SpatialTransform(
        const RigidBodyDynamics::Math::SpatialTransform& other);

    ///
    /// \brief operator= For submatrices
    /// \param other The SpatialTransform to copy
    ///
    void operator=(
        const SpatialTransform& other);
#endif

    /// 
    /// \brief Return the transformation matrix, i.e. the rototrans matrix
    /// 
    utils::RotoTrans rototrans() const;

    /// 
    /// \brief Return the rotation part of the transform
    /// 
    utils::Matrix3d rotation() const;

    /// 
    /// \brief Return the translation part of the transform
    /// 
    utils::Vector3d translation() const;

};

}
}

#endif // BIORBD_UTILS_SPATIAL_TRANSFORM_H
