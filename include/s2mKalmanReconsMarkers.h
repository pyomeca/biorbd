#ifndef KALMANRECONSMARKERS_HPP
#define KALMANRECONSMARKERS_HPP

#include "biorbdConfig.h"
#include "s2mKalmanRecons.h"

class s2mMarkers;
class s2mNodeBone;
class BIORBD_API s2mKalmanReconsMarkers : public s2mKalmanRecons
{
public:

    // Constructeur
    s2mKalmanReconsMarkers(
            s2mMusculoSkeletalModel&,
            s2mKalmanRecons::s2mKalmanParam = s2mKalmanRecons::s2mKalmanParam());
    virtual ~s2mKalmanReconsMarkers();

    // Reconstruction d'un frame
    virtual void reconstructFrame(
            s2mMusculoSkeletalModel &m,
            const s2mMarkers &Tobs,
            biorbd::utils::GenCoord *Q,
            biorbd::utils::GenCoord *Qdot,
            biorbd::utils::GenCoord *Qddot,
            bool removeAxes=true);
    virtual void reconstructFrame(
            s2mMusculoSkeletalModel &m,
            const std::vector<s2mNodeBone> &Tobs,
            biorbd::utils::GenCoord *Q,
            biorbd::utils::GenCoord *Qdot,
            biorbd::utils::GenCoord *Qddot,
            bool removeAxes=true);
    virtual void reconstructFrame(
            s2mMusculoSkeletalModel &m,
            const Eigen::VectorXd &Tobs,
            biorbd::utils::GenCoord *Q = nullptr,
            biorbd::utils::GenCoord *Qdot = nullptr,
            biorbd::utils::GenCoord *Qddot = nullptr,
            bool removeAxes=true); // Faire la reconstruction cinematique
    virtual void reconstructFrame();

    bool first();

protected:
    virtual void initialize();
    virtual void manageOcclusionDuringIteration(
            s2mMatrix&,
            Eigen::VectorXd &measure,
            const std::vector<unsigned int> &occlusion);
    s2mMatrix m_PpInitial; // Se souvenir de Pp inital
    bool m_firstIteration;
};

#endif // KALMANRECONSMARKERS_HPP
