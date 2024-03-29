#ifndef BIORBD_MUSCLES_WRAPPING_HALF_CYLINDER_H
#define BIORBD_MUSCLES_WRAPPING_HALF_CYLINDER_H

#include "biorbdConfig.h"
#include "InternalForces/WrappingObject.h"

namespace BIORBD_NAMESPACE
{
namespace internal_forces
{
///
/// \brief Half cylinder of infinite length (the length only affect the graphical
/// representation) object that makes the muscle to wrap around
///
/// Watch out, due to high computational cost, the CasADi backend always assume
/// contact between the muscle and cylinder. This will create discrepancies between
/// the Eigen and the CasADi backend
///
class BIORBD_API WrappingHalfCylinder : public WrappingObject
{
public:
    ///
    /// \brief Construct a wrapping half cylinder
    ///
    WrappingHalfCylinder();

    ///
    /// \brief Construct a wrapping half cylinder
    ///
    WrappingHalfCylinder(
        const utils::Vector3d& other);

#ifndef SWIG
    ///
    /// \brief Construct a wrapping half cylinder
    ///
    WrappingHalfCylinder(
        const utils::Vector3d* other);
#endif

    ///
    /// \brief Construct a wrapping half cylinder
    /// \param rt RotoTrans matrix of the origin of the half cylinder
    /// \param radius The radius of the half cylinder
    /// \param length The Length of the half cylinder
    ///
    WrappingHalfCylinder(
        const utils::RotoTrans& rt,
        const utils::Scalar& radius,
        const utils::Scalar& length);

    ///
    /// \brief Construct a wrapping half cylinder
    /// \param rt RotoTrans matrix
    /// \param radius The radius of the half cylinder
    /// \param length The length of the half cylinder
    /// \param name The name of the half cylinder
    /// \param parentName The parent name segment
    ///
    WrappingHalfCylinder(
        const utils::RotoTrans& rt,
        const utils::Scalar& radius,
        const utils::Scalar& length,
        const utils::String& name,
        const utils::String& parentName);

    ///
    /// \brief Deep copy of the wrapping half cylinder
    /// \return A deep copy of the wrapping half cylinder
    ///
    WrappingHalfCylinder DeepCopy() const;

    ///
    /// \brief Deep copy of the wrapping half cylinder in another wrapping half cylinder
    /// \param other The wrapping half cylinder to copy
    ///
    void DeepCopy(
        const WrappingHalfCylinder& other);

    ///
    /// \brief From the position of the half cylinder, return the 2 locations where the muscle leaves the wrapping object
    /// \param rt RotoTrans matrix of the half cylinder
    /// \param p1_bone 1st position of the muscle node
    /// \param p2_bone 2n position of the muscle node
    /// \param p1 The 1st position on the half cylinder the muscle leave
    /// \param p2 The 2nd position on the half cylinder the muscle leave
    /// \param length Length of the muscle (ignored if no value is provided)
    ///
    void wrapPoints(
        const utils::RotoTrans& rt,
        const utils::Vector3d& p1_bone,
        const utils::Vector3d& p2_bone,
        utils::Vector3d& p1,
        utils::Vector3d& p2,
        utils::Scalar* length = nullptr);

    ///
    /// \brief From the position of the half cylinder, return the 2 locations where the muscle leaves the wrapping object
    /// \param model The joint model
    /// \param Q The generalized coordinates
    /// \param p1_bone 1st position of the muscle node
    /// \param p2_bone 2n position of the muscle node
    /// \param p1 The 1st position on the half cylinder the muscle leave
    /// \param p2 The 2nd position on the half cylinder the muscle leave
    /// \param length Length of the muscle (ignored if no value is provided)
    ///
    void wrapPoints(
        rigidbody::Joints& model,
        const rigidbody::GeneralizedCoordinates& Q,
        const utils::Vector3d& p1_bone,
        const utils::Vector3d& p2_bone,
        utils::Vector3d& p1,
        utils::Vector3d& p2,
        utils::Scalar* length = nullptr) ;

    ///
    /// \brief Returns the previously computed 2 locations where the muscle leaves the wrapping object
    /// \param p1 The 1st position on the half cylinder the muscle leave
    /// \param p2 The 2nd position on the half cylinder the muscle leave
    /// \param length Length of the muscle (ignored if no value is provided)
    ///
    void wrapPoints(
        utils::Vector3d& p1,
        utils::Vector3d& p2,
        utils::Scalar* length = nullptr);

    ///
    /// \brief Return the RotoTrans matrix of the half cylinder
    /// \param model The joint model
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics should be computed
    /// \return The RotoTrans matrix of the half cylinder
    ///
    virtual const utils::RotoTrans& RT(
        rigidbody::Joints &model,
        const rigidbody::GeneralizedCoordinates& Q,
        bool updateKin = true);

    ///
    /// \brief Set the diameter of the wrapping half cylinder
    /// \param val Value of the diameter to set
    ///
    void setRadius(
        const utils::Scalar& val);

    ///
    /// \brief Return the radius of the half cylinder
    /// \return The radius of the half cylinder
    ///
    utils::Scalar radius() const;

    ///
    /// \brief Return the diamter of the half cylinder
    /// \return The diamter of the half cylinder
    ///
    utils::Scalar diameter() const;

    ///
    /// \brief Set the length of the half cylinder
    /// \param val Value of the to set
    ///
    void setLength(
        const utils::Scalar& val);

    ///
    /// \brief Return the length of the half cylinder
    /// \return The length of the half cylinder
    ///
    const utils::Scalar& length() const;

protected:
#ifndef SWIG
    ///
    /// \brief Pair of 2 muscles points
    ///
    class NodeMusclePair
    {
    public:
        ///
        /// \brief Construct a node muscle pair
        /// \param p1 Point 1
        /// \param p2 Point 2
        ///
        NodeMusclePair(
            const utils::Vector3d &p1,
            const utils::Vector3d &p2) :
            m_p1(std::make_shared<utils::Vector3d>(p1)),
            m_p2(std::make_shared<utils::Vector3d>(p2))
        {}
        std::shared_ptr<utils::Vector3d> m_p1; ///< Point 1
        std::shared_ptr<utils::Vector3d> m_p2; ///< Point 2
    };
#endif

    ///
    /// \brief Find the two tangents of a point with a circle
    /// \param p The point
    /// \param p_tan The point tangent
    ///
    void findTangentToCircle(
        const utils::Vector3d& p,
        utils::Vector3d& p_tan) const;

    ///
    /// \brief Select between a set of nodes which ones to keep
    /// \param p The 2 muscles points
    /// \param p_tan The selected point
    ///
    void selectTangents(
        const NodeMusclePair&p,
        utils::Vector3d& p_tan) const;

    ///
    /// \brief Find the height of both points
    /// \param pointsInGlobal The position of the muscle pair in global reference frame
    /// \param pointsToWrap The points to wrap
    /// \return Return false if no wrap is needed
    ///
    bool findVerticalNode(
        const NodeMusclePair& pointsInGlobal,
        NodeMusclePair& pointsToWrap) const;

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Check if a wrapper has to be done
    /// \param pointsInGlobal The position of the muscle pair in global reference frame
    /// \param pointsToWrap The points to wrap
    /// \return If the wrapper has to be done
    ///
    bool checkIfWraps(
        const NodeMusclePair &pointsInGlobal,
        NodeMusclePair &pointsToWrap) const;
#endif

    ///
    /// \brief Compute the muscle length on the half cylinder
    /// \param p the muscle node pair
    /// \return The muscle lengh on the half cylinder
    ///
    utils::Scalar computeLength(
        const NodeMusclePair &p) const;

    std::shared_ptr<utils::Scalar>
    m_radius; ///< Diameter of the half cylinder diametre du cylindre
    std::shared_ptr<utils::Scalar>
    m_length; ///< Length of the half cylinder
    std::shared_ptr<utils::RotoTrans>
    m_RTtoParent; ///<RotoTrans matrix with the parent

    std::shared_ptr<utils::Vector3d>
    m_p1Wrap; ///< First point of contact with the wrap
    std::shared_ptr<utils::Vector3d>
    m_p2Wrap; ///< Second point of contact with the wrap
    std::shared_ptr<utils::Scalar> m_lengthAroundWrap
    ; ///< Length between p1 and p2

};

}
}

#endif // BIORBD_MUSCLES_WRAPPING_HALF_CYLINDER_H

