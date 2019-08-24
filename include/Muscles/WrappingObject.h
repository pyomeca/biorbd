#ifndef BIORBD_MUSCLES_WRAPPING_OBJECT_H
#define BIORBD_MUSCLES_WRAPPING_OBJECT_H

#include "biorbdConfig.h"
#include "Utils/String.h"
#include "Muscles/PathChanger.h"

namespace biorbd {
namespace rigidbody {
class Joints;
class GeneralizedCoordinates;
}

namespace muscles {

class BIORBD_API WrappingObject : public biorbd::muscles::PathChanger
{
public:
    WrappingObject(
            double x,
            double y,
            double z,
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    WrappingObject(
            const biorbd::utils::Node3d &v, // Position du noeud
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    virtual ~WrappingObject() ;

    virtual const biorbd::utils::RotoTrans& RT(
            biorbd::rigidbody::Joints &m,
            const biorbd::rigidbody::GeneralizedCoordinates& Q,
            bool  = true) = 0;
    virtual void wrapPoints(
            const biorbd::utils::RotoTrans&,
            const biorbd::muscles::MuscleNode&,
            const biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&, double* = nullptr) = 0 ; // Premier et dernier points musculaire
    virtual void wrapPoints(
            biorbd::rigidbody::Joints&,
            const biorbd::rigidbody::GeneralizedCoordinates&,
            const biorbd::muscles::MuscleNode&,
            const biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            double* = nullptr) = 0; // Premier et dernier points musculaire
    virtual void wrapPoints(
            biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            double* = nullptr) = 0; // Assume un appel déja faits

    // Set and get
    const biorbd::utils::String& forme() const;

protected:
    biorbd::utils::String m_forme;

};

}}

#endif // BIORBD_MUSCLES_WRAPPING_OBJECT_H
