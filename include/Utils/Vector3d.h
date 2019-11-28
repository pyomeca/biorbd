#ifndef BIORBD_UTILS_Vector3d_H
#define BIORBD_UTILS_Vector3d_H

#include <memory>
#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/Node.h"

namespace biorbd {
namespace utils {
class RotoTrans;
class String;
///
/// \brief Class Vector3d
///
class BIORBD_API Vector3d : public Eigen::Vector3d, public biorbd::utils::Node
{
    public:
    ///
    /// \brief Construct 3D vector
    /// 
    Vector3d();
    ///
    /// \brief Construct 3D vector
    /// \param x Position on the x axis
    /// \param y Position on the y axis
    /// \param z Position on the z axis
    ///
    Vector3d(
            double x,
            double y,
            double z);
    ///
    /// \brief Construct a 3D vector from an eigen 4D vector
    /// \param v The Eigen 4D vector
    ///
    Vector3d(const Eigen::Vector4d& v);

    ///
    /// \brief Construct a 3D vector
    /// \param other Position of the node (eigen matrix)
    ///
    template<typename OtherDerived> Vector3d(const Eigen::MatrixBase<OtherDerived>& other) :
        Eigen::Vector3d(other), biorbd::utils::Node () {}

    ///
    /// \brief Construct a 3D vector
    /// \param other Position of the vector (eigen matrix)
    /// \param name Name of the vector
    /// \param parentName The name of the parent segment
    /// 
    template<typename OtherDerived> Vector3d(
            const Eigen::MatrixBase<OtherDerived>& other, 
            const biorbd::utils::String &name,  
            const biorbd::utils::String &parentName) :
        Eigen::Vector3d(other), biorbd::utils::Node (name, parentName) {}

    ///
    /// \brief Construct a 3D vector
    /// \param x Component on the x axis
    /// \param y Component on the y axis
    /// \param z Component on the z axis
    /// \param name Name of the vector
    /// \param parentName Name of the parent segment
    ///
    Vector3d(
            double x,
            double y,
            double z, 
            const biorbd::utils::String &name, 
            const biorbd::utils::String &parentName);

    ///
    /// \brief Deep copy of a 3D vector
    /// \return A deep copy of a 3D vector
    ///
    biorbd::utils::Vector3d DeepCopy() const;
    
    ///
    /// \brief Deep copy of a 3D vector into another 3D vector
    /// \param other The 3D vector to copy
    ///
    void DeepCopy(const biorbd::utils::Vector3d& other);

    // Get and Set
    ///
    /// \brief Apply RT to 3D vector
    /// \param rt RotoTrans to apply
    /// \return The 3D vector
    //
    biorbd::utils::Vector3d applyRT(const RotoTrans&rt) const;
    ///
    /// \brief Apply RT to 3D vector
    /// \param rt Rototrans to apply
    ///
    void applyRT(const RotoTrans&rt);

    ///
    /// \brief To use operator "=" on 3D vector with eigen 4D vector
    /// \param other The eigen 4D vector
    ///
    biorbd::utils::Vector3d& operator=(const Eigen::Vector4d& other);
    ///
    /// \brief To use operator "=" on 3D vector with eigen matrix
    /// \param other The eigen matrix
    ///
    template<typename OtherDerived>
        biorbd::utils::Vector3d& operator=(const Eigen::MatrixBase <OtherDerived>& other){
            this->Eigen::Vector3d::operator=(other);
            return *this;
        }

protected:
    ///
    /// \brief Set the type Vector3d
    ///
    void setType();
};

}}

#endif // BIORBD_UTILS_VECTOR3D_H
