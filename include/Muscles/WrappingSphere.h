#ifndef BIORBD_MUSCLES_WRAPPING_SPHERE_H
#define BIORBD_MUSCLES_WRAPPING_SPHERE_H

#include "biorbdConfig.h"
#include "Muscles/WrappingObject.h"

namespace biorbd {
namespace muscles {

class BIORBD_API WrappingSphere : public biorbd::muscles::WrappingObject
{
public:
    WrappingSphere(
            double x,
            double y,
            double z,
            double diameter, // Diametre de la sphere
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    WrappingSphere(
            const biorbd::utils::Node &v, // Position du noeud
            double diameter, // Diametre de la sphere
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    virtual ~WrappingSphere();

    biorbd::utils::Attitude RT(
            biorbd::rigidbody::Joints &,
            const biorbd::rigidbody::GeneralizedCoordinates& ,
            bool  = true);
    virtual void wrapPoints(
            const biorbd::utils::Attitude&,
            const biorbd::muscles::MuscleNode&,
            const biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&, double* = nullptr) {} // Premier et dernier points musculaire
    virtual void wrapPoints(
            biorbd::rigidbody::Joints&,
            const biorbd::rigidbody::GeneralizedCoordinates&,
            const biorbd::muscles::MuscleNode&,
            const biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            double* = nullptr) {} // Premier et dernier points musculaire
    virtual void wrapPoints(
            biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            double* = nullptr) {} // Premier et dernier points musculaire

    // Get and set
    double size() const;
    void setSize(double val);
protected:
    double m_dia;

};

}}

#endif // BIORBD_MUSCLES_WRAPPING_SPHERE_H
