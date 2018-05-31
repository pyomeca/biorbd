#ifndef KALMANRECONSIMU_HPP
#define KALMANRECONSIMU_HPP

#include "biorbdConfig.h"
#include "s2mKalmanRecons.h"


class BIORBD_API s2mKalmanReconsIMU : public s2mKalmanRecons
{
public:

    // Constructeur
    s2mKalmanReconsIMU(s2mMusculoSkeletalModel&, s2mKalmanRecons::s2mKalmanParam = s2mKalmanRecons::s2mKalmanParam(100, 0.005, 1e-10));

    // Reconstruction d'un frame
    virtual void reconstructFrame(s2mMusculoSkeletalModel &m, const std::vector<s2mAttitude> &IMUobs, s2mGenCoord *Q, s2mGenCoord *Qdot, s2mGenCoord *Qddot);
    virtual void reconstructFrame(s2mMusculoSkeletalModel &m, const Eigen::VectorXd &IMUobs, s2mGenCoord *Q, s2mGenCoord *Qdot, s2mGenCoord *Qddot);
    virtual void reconstructFrame(){s2mError::s2mAssert(false, "Implémentation impossible");}

    bool first();

protected:
    virtual void initialize();
    virtual void manageOcclusionDuringIteration(Eigen::MatrixXd&, Eigen::VectorXd &measure, const std::vector<unsigned int> &occlusion);

    Eigen::MatrixXd m_PpInitial; // Se souvenir de Pp inital
    bool m_firstIteration;
};

#endif // KALMANRECONSIMU_HPP
