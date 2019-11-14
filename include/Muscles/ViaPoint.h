#ifndef BIORBD_MUSCLES_VIAPOINT_H
#define BIORBD_MUSCLES_VIAPOINT_H

#include "biorbdConfig.h"
#include "Utils/Node3d.h"

namespace biorbd {
namespace utils {
class String;
}

namespace muscles {
///
    /// \brief Class ViaPoint
    ///
class BIORBD_API ViaPoint : public biorbd::utils::Node3d{
public:
    ///
    /// \brief Contruct ViaPoint
    ///
    ViaPoint();
    ///
    /// \brief Construct ViaPoint
    /// \param x Position of point on x axis
    /// \param y Position of point on y axis
    /// \param z Position of point on z axis
    ///
    ViaPoint(
            double x,
            double y,
            double z);
    ///
    /// \brief Construct ViaPoint
    /// \param x Position of point on x axis
    /// \param y Position of point on y axis
    /// \param z Position of point on z axis
    /// \param name The name of the node
    /// \param parentName The name of the parent
    ///
    ViaPoint(
            double x,
            double y,
            double z,
            const biorbd::utils::String &name,  
            const biorbd::utils::String &parentName);

    ///
    /// \brief Construct ViaPoint from a 3D node
    /// \param other The 3D node
    ///
    ViaPoint(
            const biorbd::utils::Node3d& other);
    ///
    /// \brief Construct ViaPoint from another ViaPoint
    /// \param other The other ViaPoint
    ///
    ViaPoint(
            const biorbd::muscles::ViaPoint& other);

    ///
    /// \brief Deep copy of a ViaPoint
    /// \return A deep copy of a ViaPoint
    ///
    biorbd::muscles::ViaPoint DeepCopy() const;
    ///
    /// \brief Deep copy of a ViaPoint into another ViaPoint
    /// \param other The ViaPoint to copy
    ///
    void DeepCopy(const biorbd::muscles::ViaPoint& other);

    ///
    /// \brief To be able to use operator "=" when assigning a ViaPoint to a 3D node
    /// \param other The 3D node to assign to ViaPoint
    ///
    template<typename OtherDerived>
        biorbd::muscles::ViaPoint& operator=(const biorbd::utils::Node3d& other){
            this->biorbd::utils::Node3d::operator=(other);
            return *this;
        }
};

}}

#endif // BIORBD_MUSCLES_VIAPOINT_H
