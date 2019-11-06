#ifndef BIORBD_MUSCLES_WRAPPING_SPHERE_H
#define BIORBD_MUSCLES_WRAPPING_SPHERE_H

#include "biorbdConfig.h"
#include "Muscles/WrappingObject.h"

namespace biorbd {
namespace muscles {
///
/// \brief Class WrappingSphere
///
class BIORBD_API WrappingSphere : public biorbd::muscles::WrappingObject
{
public:

    ///
    /// \brief Construct a wrapping sphere
    ///
    WrappingSphere();
    ///
    /// \brief Construct a wrapping sphere
    /// \param x Position on x axis
    /// \param y Position on y axis
    /// \param z Position on z axis
    /// \param diameter Diameter of the sphere
    ///
    WrappingSphere(
            double x,
            double y,
            double z,
            double diameter);
    ///
    /// \brief Construct a wrapping sphere
    /// \param x Position on x axis
    /// \param y Position on y axis
    /// \param z Position on z axis
    /// \param diameter Diameter of the sphere
    /// \param name Name of the node
    /// \param parentName Name of the parent
    ///
    WrappingSphere(
            double x,
            double y,
            double z,
            double diameter, 
            const biorbd::utils::String &name, 
            const biorbd::utils::String &parentName);
    ///
    /// \brief Construct a wrapping sphere
    /// \param v Position of the node
    /// \param diameter Diameter of the sphere
    ///
    WrappingSphere(
            const biorbd::utils::Node3d &v, 
            double diameter);
    ///
    /// \brief Deep copy of the wrapping sphere
    /// \return A deep copy of the wrapping sphere
    ///
    biorbd::muscles::WrappingSphere DeepCopy() const;
    ///
    /// \brief Deep copy of the wrapping sphere in another wrapping sphere
    /// \param other The wrapping sphere to copy
    ///
    void DeepCopy(const biorbd::muscles::WrappingSphere& other);

    ///
    /// \brief Return the RotoTrans matrix of the wrapping sphere
    /// \param model The model
    /// \param Q The position variables
    /// \param updateKin Update kinematics (default: True)
    /// \return The RotoTrans matrix of the wrapping sphere
    ///
    const biorbd::utils::RotoTrans& RT(
            biorbd::rigidbody::Joints &model,
            const biorbd::rigidbody::GeneralizedCoordinates&Q ,
            bool updateKin  = true);
    ///
    /// \brief This function takes the position of the wrapping and finds the location where muscle 1 and 2 leave the wrapping object
    /// \param rt RotoTrans matrix
    /// \param p1_bone TODO
    /// \param p2_bone TODO
    /// \param p1 TODO
    /// \param p2 TODO
    /// \param length Length of the muscle (default: nullptr)
    ///
    virtual void wrapPoints(
            const biorbd::utils::RotoTrans&rt,
            const biorbd::utils::Node3d&p1_bone,
            const biorbd::utils::Node3d&p2_bone,
            biorbd::utils::Node3d&p1,
            biorbd::utils::Node3d&p2, double* length= nullptr) {} 

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
    virtual void wrapPoints(
            biorbd::rigidbody::Joints&model,
            const biorbd::rigidbody::GeneralizedCoordinates&Q,
            const biorbd::utils::Node3d&p1_bone,
            const biorbd::utils::Node3d&p2_bone,
            biorbd::utils::Node3d&p1,
            biorbd::utils::Node3d&p2,
            double* length = nullptr) {}

    ///
    /// \brief This function takes finds the location where muscle 1 and 2 leave the wrapping object (if already computed)
    /// \param p1
    /// \param p2
    /// \param length Length of the muscle (default: nullptr)
    ///
    virtual void wrapPoints(
            biorbd::utils::Node3d&p1,
            biorbd::utils::Node3d&p2,
            double* length= nullptr) {} 

    // Get and set
    ///
    /// \brief Return the diameter of the wrapping sphere
    /// \return The diameter of the wrapping sphere
    ///
    double size() const;

    ///
    /// \brief Set the diameter of the wrappping sphere
    /// \param val Value of the diameter
    ///
    void setSize(double val);
protected:
    std::shared_ptr<double> m_dia; ///< Diameter of the wrapping sphere

};

}}

#endif // BIORBD_MUSCLES_WRAPPING_SPHERE_H
