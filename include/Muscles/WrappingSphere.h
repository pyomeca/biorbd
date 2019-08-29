#ifndef BIORBD_MUSCLES_WRAPPING_SPHERE_H
#define BIORBD_MUSCLES_WRAPPING_SPHERE_H

#include "biorbdConfig.h"
#include "Muscles/WrappingObject.h"

namespace biorbd {
namespace muscles {

class BIORBD_API WrappingSphere : public biorbd::muscles::WrappingObject
{
public:
    WrappingSphere();
    WrappingSphere(
            double x,
            double y,
            double z,
            double diameter);
    WrappingSphere(
            double x,
            double y,
            double z,
            double diameter, // Diametre de la sphere
            const biorbd::utils::String &name, // Nom du noeud
            const biorbd::utils::String &parentName);
    WrappingSphere(
            const biorbd::utils::Node3d &v, // Position du noeud
            double diameter);
    biorbd::muscles::WrappingSphere DeepCopy() const;
    void DeepCopy(const biorbd::muscles::WrappingSphere& other);

    const biorbd::utils::RotoTrans& RT(
            biorbd::rigidbody::Joints &,
            const biorbd::rigidbody::GeneralizedCoordinates& ,
            bool  = true);
    virtual void wrapPoints(
            const biorbd::utils::RotoTrans&,
            const biorbd::utils::Node3d&,
            const biorbd::utils::Node3d&,
            biorbd::utils::Node3d&,
            biorbd::utils::Node3d&, double* = nullptr) {} // Premier et dernier points musculaire
    virtual void wrapPoints(
            biorbd::rigidbody::Joints&,
            const biorbd::rigidbody::GeneralizedCoordinates&,
            const biorbd::utils::Node3d&,
            const biorbd::utils::Node3d&,
            biorbd::utils::Node3d&,
            biorbd::utils::Node3d&,
            double* = nullptr) {} // Premier et dernier points musculaire
    virtual void wrapPoints(
            biorbd::utils::Node3d&,
            biorbd::utils::Node3d&,
            double* = nullptr) {} // Premier et dernier points musculaire

    // Get and set
    double size() const;
    void setSize(double val);
protected:
    std::shared_ptr<double> m_dia;

};

}}

#endif // BIORBD_MUSCLES_WRAPPING_SPHERE_H
