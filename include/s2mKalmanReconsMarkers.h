#ifndef KALMANRECONSMARKERS_HPP
#define KALMANRECONSMARKERS_HPP

#include "biorbdConfig.h"
#include "s2mKalmanRecons.h"

class BIORBD_API s2mKalmanReconsMarkers : public s2mKalmanRecons
{
public:

    // Constructeur
    s2mKalmanReconsMarkers(s2mMusculoSkeletalModel&, s2mKalmanRecons::s2mKalmanParam = s2mKalmanRecons::s2mKalmanParam());

    // Reconstruction d'un frame
    virtual void reconstructFrame(s2mMusculoSkeletalModel &m, const s2mMarkers &Tobs, s2mGenCoord *Q, s2mGenCoord *Qdot, s2mGenCoord *Qddot, bool removeAxes=true);
    virtual void reconstructFrame(s2mMusculoSkeletalModel &m, const std::vector<Eigen::Vector3d> &Tobs, s2mGenCoord *Q, s2mGenCoord *Qdot, s2mGenCoord *Qddot, bool removeAxes=true);
    virtual void reconstructFrame(s2mMusculoSkeletalModel &m, const Eigen::VectorXd &Tobs, s2mGenCoord *Q = NULL, s2mGenCoord *Qdot = NULL, s2mGenCoord *Qddot = NULL, bool removeAxes=true); // Faire la reconstruction cinematique
    virtual void reconstructFrame(){s2mError::s2mAssert(false, "Implémentation impossible");}

    bool first();

protected:
    virtual void initialize();
    virtual void manageOcclusionDuringIteration(Eigen::MatrixXd&, Eigen::VectorXd &measure, const std::vector<unsigned int> &occlusion);
    Eigen::MatrixXd m_PpInitial; // Se souvenir de Pp inital
    bool m_firstIteration;
};

#endif // KALMANRECONSMARKERS_HPP
