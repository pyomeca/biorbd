#ifndef S2M_WRAPPING_SPHERE_H
#define S2M_WRAPPING_SPHERE_H

#include "biorbdConfig.h"
#include "s2mWrappingObject.h"

class BIORBD_API s2mWrappingSphere : public s2mWrappingObject
{
public:
    s2mWrappingSphere(
            const double &dia = 0, // Diametre de la sphere
            const Eigen::Vector3d &v = Eigen::Vector3d(0,0,0), // Position du noeud
            const biorbd::utils::String &name = "",  // Nom du noeud
            const biorbd::utils::String &parentName = "");
    virtual ~s2mWrappingSphere();

    biorbd::utils::Attitude RT(
            s2mJoints &,
            const biorbd::utils::GenCoord& ,
            const bool & = true);
    virtual void wrapPoints(
            const biorbd::utils::Attitude&,
            const s2mNodeMuscle&,
            const s2mNodeMuscle&,
            s2mNodeMuscle&,
            s2mNodeMuscle&, double* = nullptr) {} // Premier et dernier points musculaire
    virtual void wrapPoints(
            s2mJoints&,
            const biorbd::utils::GenCoord&,
            const s2mNodeMuscle&,
            const s2mNodeMuscle&,
            s2mNodeMuscle&,
            s2mNodeMuscle&,
            double* = nullptr) {} // Premier et dernier points musculaire
    virtual void wrapPoints(
            s2mNodeMuscle&,
            s2mNodeMuscle&,
            double* = nullptr) {} // Premier et dernier points musculaire

    // Get and set
    double size() const;
    void setSize(const double &val);
protected:
    double m_dia;

};

#endif // S2M_WRAPPING_SPHERE_H
