#ifndef S2M_WRAPPING_OBJECT_H
#define S2M_WRAPPING_OBJECT_H

#include <Eigen/Dense>
#include "biorbdConfig.h"
#include "s2mMusclePathChanger.h"
#include "Utils/String.h"

class s2mJoints;
namespace biorbd { namespace utils {
class GenCoord;
}}
class BIORBD_API s2mWrappingObject : public s2mMusclePathChanger
{
public:
    s2mWrappingObject(
            const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    virtual ~s2mWrappingObject() ;

    virtual biorbd::utils::Attitude RT(
            s2mJoints &m,
            const biorbd::utils::GenCoord& Q,
            const bool & = true) = 0;
    virtual void wrapPoints(
            const biorbd::utils::Attitude&,
            const s2mNodeMuscle&,
            const s2mNodeMuscle&,
            s2mNodeMuscle&,
            s2mNodeMuscle&, double* = nullptr) = 0 ; // Premier et dernier points musculaire
    virtual void wrapPoints(
            s2mJoints&,
            const biorbd::utils::GenCoord&,
            const s2mNodeMuscle&,
            const s2mNodeMuscle&,
            s2mNodeMuscle&,
            s2mNodeMuscle&,
            double* = nullptr) = 0; // Premier et dernier points musculaire
    virtual void wrapPoints(
            s2mNodeMuscle&,
            s2mNodeMuscle&,
            double* = nullptr) = 0; // Assume un appel déja faits

    // Set and get
    const biorbd::utils::String& forme() const;

protected:
    biorbd::utils::String m_forme;

};

#endif // S2M_WRAPPING_OBJECT_H
