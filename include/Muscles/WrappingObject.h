#ifndef BIORBD_MUSCLES_WRAPPING_OBJECT_H
#define BIORBD_MUSCLES_WRAPPING_OBJECT_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "Utils/String.h"
#include "Muscles/PathChanger.h"

namespace biorbd {
namespace rigidbody {
class Joints;
}

namespace utils {
class GenCoord;
}

namespace muscles {

class BIORBD_API WrappingObject : public biorbd::muscles::PathChanger
{
public:
    WrappingObject(
            const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    virtual ~WrappingObject() ;

    virtual biorbd::utils::Attitude RT(
            biorbd::rigidbody::Joints &m,
            const biorbd::utils::GenCoord& Q,
            const bool & = true) = 0;
    virtual void wrapPoints(
            const biorbd::utils::Attitude&,
            const biorbd::muscles::MuscleNode&,
            const biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&,
            biorbd::muscles::MuscleNode&, double* = nullptr) = 0 ; // Premier et dernier points musculaire
    virtual void wrapPoints(
            biorbd::rigidbody::Joints&,
            const biorbd::utils::GenCoord&,
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
