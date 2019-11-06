#ifndef BIORBD_MUSCLES_WRAPPING_CYLINDER_H
#define BIORBD_MUSCLES_WRAPPING_CYLINDER_H

#include "biorbdConfig.h"
#include "Muscles/WrappingObject.h"

namespace biorbd {
namespace muscles {
///
/// \brief Class WrappingCylinder
///
class BIORBD_API WrappingCylinder : public biorbd::muscles::WrappingObject
{
public:
    ///
    /// \brief Construct a wrapping cylinder
    ///
    WrappingCylinder();
    ///
    /// \brief Construct a wrapping cylinder
    /// \param rt RotoTrans matrix
    /// \param diameter Diameter of the cylinder
    /// \param length Length of the cylinder
    /// \param isCylinderPositiveSign If cylinder is of positive sign
    ///
    WrappingCylinder(
            const biorbd::utils::RotoTrans& rt,
            double diameter,
            double length,
            bool isCylinderPositiveSign);
    ///
    /// \brief Construct a wrapping cylinder
    /// \param rt RotoTrans matrix
    /// \param diameter Diameter of the cylinder
    /// \param length Length of the cylinder
    /// \param isCylinderPositiveSign If cylinder is of positive sign
    /// \param name The name
    /// \param parentName The parent name
    ///
    WrappingCylinder(
            const biorbd::utils::RotoTrans& rt,
            double diameter,
            double length,
            bool isCylinderPositiveSign,
            const biorbd::utils::String& name,
            const biorbd::utils::String& parentName);

    ///
    /// \brief Deep copy of the wrapping cylinder
    /// \return A deep copy of the wrapping cylinder
    ///
    biorbd::muscles::WrappingCylinder DeepCopy() const;
    ///
    /// \brief Deep copy of the wrapping cylinder in another wrapping cylinder
    /// \param other The wrapping cylinder to copy
    ///
    void DeepCopy(const biorbd::muscles::WrappingCylinder& other);

    ///
    /// \brief This function takes the position of the wrapping and finds the location where muscle 1 and 2 leave the wrapping object
    /// \param rt RotoTrans matrix
    /// \param p1_bone TODO
    /// \param p2_bone TODO
    /// \param p1 TODO
    /// \param p2 TODO
    /// \param length Length of the muscle (default: nullptr)
    ///
    void wrapPoints(
            const biorbd::utils::RotoTrans& rt,
            const biorbd::utils::Node3d& p1_bone,
            const biorbd::utils::Node3d& p2_bone,
            biorbd::utils::Node3d& p1,
            biorbd::utils::Node3d& p2,
            double* length = nullptr); 

    ///
    /// \brief This function takes a model and a position and finds the location where muscle 1 and 2 leave the wrapping object
    /// \param model The model
    /// \param Q The position variables 
    /// \param p1_bone
    /// \param p2_bone
    /// \param p1
    /// \param p2
    /// \param length Length of the muscle (default: nullptr)
    ///
    void wrapPoints(
            biorbd::rigidbody::Joints& model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            const biorbd::utils::Node3d& p1_bone,
            const biorbd::utils::Node3d& p2_bone,
            biorbd::utils::Node3d& p1,
            biorbd::utils::Node3d& p2,
            double* length = nullptr) ; 
    ///
    /// \brief This function takes finds the location where muscle 1 and 2 leave the wrapping object (if already computed)
    /// \param p1
    /// \param p2
    /// \param length Length of the muscle (default: nullptr)
    ///
    void wrapPoints(
            biorbd::utils::Node3d& p1,
            biorbd::utils::Node3d& p2,
            double* length = nullptr); 

    // Set et get
    ///
    /// \brief Return the RotoTrans matrix of the wrapping cylinder
    /// \param model The model
    /// \param Q The position variables
    /// \param updateKin Update kinematics (default: True)
    /// \return The RotoTrans matrix of the wrapping cylinder
    ///
    virtual const biorbd::utils::RotoTrans& RT(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            bool updateKin = true);

    ///
    /// \brief Return the diameter of the wrapping cylinder
    /// \return The diameter of the wrapping cylinder
    ///
    double diameter() const;

    ///
    /// \brief Set the diameter of the wrapping cylinder
    /// \param val Value of the diameter to set
    ///
    void setDiameter(double val);

    ///
    /// \brief Return the radius of the wrapping cylinder
    /// \return The radius of the wrapping cylinder
    ///
    double rayon() const;
    ///
    /// \brief Return the length of the wrapping cylinder
    /// \return The length of the wrapping cylinder
    ///
    double length() const;
    ///
    /// \brief Set the length of the wrapping cylinder
    /// \param val Value of the length to set
    ///
    void setLength(double val);

protected:
    ///
    /// \brief Class NodeMusclePair
    ///
    class NodeMusclePair{
    public:
        ///
        /// \brief Construct a node muscle pair
        /// \param p1 Point 1
        /// \param p2 Point 2
        ///
        NodeMusclePair(
                const biorbd::utils::Node3d &p1,
                const biorbd::utils::Node3d &p2) :
            m_p1(std::make_shared<biorbd::utils::Node3d>(p1)),
            m_p2(std::make_shared<biorbd::utils::Node3d>(p2))
        {}
        std::shared_ptr<biorbd::utils::Node3d> m_p1; ///< Point 1
        std::shared_ptr<biorbd::utils::Node3d> m_p2;///< Point 2
    };

 
    ///
    /// \brief Find the two tangents of a point with a circle
    /// \param p The point
    /// \param p_tan The point tangent
    ///
    void findTangentToCircle(
            const biorbd::utils::Node3d& p,
            biorbd::utils::Node3d&p_tan) const;
    
    ///
    /// \brief Select between a set of nodes which ones to keep
    /// \param p TODO
    /// \param p_tan TODO
    /// 
    void selectTangents(
            const NodeMusclePair&p, 
        biorbd::utils::Node3d&p_tan) const;

    ///
    /// \brief Find the height of both points
    /// \param glob TODO
    /// \param wrapper The wrap
    /// \return True or False (false = no wrap)
    ///
    bool findVerticalNode(
            const NodeMusclePair&glob,
            NodeMusclePair&wrapper) const;
    // Savoir s'il y a un wrapping qui doit etre fait

    ///
    /// \brief To know if a wrapper has to be done
    /// \param glob
    /// \param wrapper The wrap
    /// \return True or false
    ///
    bool checkIfWraps(
            const NodeMusclePair &glob,
            NodeMusclePair &wrapper) const;

    // Calcul de la longueur musculaire sur le cylindre
    ///
    /// \brief Compute the muscle length on the cylinder
    /// \param p TODO
    /// \return The muscle lengh on the cylinder
    ///
    double computeLength(
            const NodeMusclePair &) const;

    std::shared_ptr<double> m_dia; ///< Diameter of the cylinder diametre du cylindre
    std::shared_ptr<double> m_length; ///< Length of the cylinder
    std::shared_ptr<bool> m_isCylinderPositiveSign; ///<orientation of the muscle passing
    std::shared_ptr<biorbd::utils::RotoTrans> m_RTtoParent; ///<RotoTrans matrix with the parent

    std::shared_ptr<biorbd::utils::Node3d> m_p1Wrap; ///< First point of contact with the wrap
    std::shared_ptr<biorbd::utils::Node3d> m_p2Wrap; ///< Second point of contact with the wrap
    std::shared_ptr<double> m_lengthAroundWrap ; ///< Length between p1 and p2

};

}}

#endif // BIORBD_MUSCLES_WRAPPING_CYLINDER_H

