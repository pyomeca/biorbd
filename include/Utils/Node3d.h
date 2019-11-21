#ifndef BIORBD_UTILS_NODE3D_H
#define BIORBD_UTILS_NODE3D_H

#include <memory>
#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/Node.h"

namespace biorbd {
namespace utils {
class RotoTrans;
class String;
///
/// \brief Class Node3D
///
class BIORBD_API Node3d : public Eigen::Vector3d, public biorbd::utils::Node
{
    public:
    ///
    /// \brief Construct 3D node
    /// 
    Node3d();
    ///
    /// \brief Construct 3D node
    /// \param x Position on the x axis
    /// \param y Position on the y axis
    /// \param z Position on the z axis
    ///
    Node3d(
            double x,
            double y,
            double z);
    ///
    /// \brief Construct a 3D node from an eigen 4D vector
    /// \param v The Eigen 4D vector
    ///
    Node3d(const Eigen::Vector4d& v);

    ///
    /// \brief Construct a 3D node
    /// \param other Position of the node (eigen matrix)
    ///
    template<typename OtherDerived> Node3d(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Vector3d(other), biorbd::utils::Node () {}

    ///
    /// \brief Construct a 3D node
    /// \param other Position of the node (eigen matrix)
    /// \param name Name of the node
    /// \param parentName The name of the parent segment
    /// 
    template<typename OtherDerived> Node3d(
            const Eigen::MatrixBase<OtherDerived>& other, 
            const biorbd::utils::String &name,  
            const biorbd::utils::String &parentName) :
        Eigen::Vector3d(other), biorbd::utils::Node (name, parentName) {}

    ///
    /// \brief Construct a 3D node
    /// \param x Position of the node on the x axis
    /// \param y Position of the node on the y axis
    /// \param z Position of the node on the z axis
    /// \param name Name of the node
    /// \param parentName Name of the parent segment
    ///
    Node3d(
            double x,
            double y,
            double z, 
            const biorbd::utils::String &name, 
            const biorbd::utils::String &parentName);

    ///
    /// \brief Deep copy of a 3D node
    /// \return A deep copy of a 3D node
    ///
    biorbd::utils::Node3d DeepCopy() const;
    
    ///
    /// \brief Deep copy of a 3D node into another 3D node
    /// \param other The 3D node to copy
    ///
    void DeepCopy(const biorbd::utils::Node3d& other);

    // Get and Set
    ///
    /// \brief Apply RT to 3D node
    /// \param rt RT to apply
    //
    biorbd::utils::Node3d applyRT(const RotoTrans&rt) const;
    void applyRT(const RotoTrans&);

    ///
    /// \brief To use operator "=" on 3D node with eigen 4D vector
    /// \param other The eigen 4D vector
    ///
    biorbd::utils::Node3d& operator=(const Eigen::Vector4d& other);
    ///
    /// \brief To use operator "=" on 3D node with eigen matrix
    /// \param other The eigen matrix
    ///
    template<typename OtherDerived>
        biorbd::utils::Node3d& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->Eigen::Vector3d::operator=(other);
            return *this;
        }

protected:
    ///
    /// \brief Set the type NODE3D
    ///
    void setType();
};

}}

#endif // BIORBD_UTILS_NODE3D_H
