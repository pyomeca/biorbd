#ifndef S2M_KALMAN_RECONS_IMU_H
#define S2M_KALMAN_RECONS_IMU_H

#include "biorbdConfig.h"
#include "s2mKalmanRecons.h"

namespace biorbd { namespace utils {
class Attitude;
}}
class BIORBD_API s2mKalmanReconsIMU : public s2mKalmanRecons
{
public:

    // Constructeur
    s2mKalmanReconsIMU(
            s2mMusculoSkeletalModel&,
            s2mKalmanRecons::s2mKalmanParam = s2mKalmanRecons::s2mKalmanParam(100, 0.005, 1e-10));
    virtual ~s2mKalmanReconsIMU();

    // Reconstruction d'un frame
    virtual void reconstructFrame(
            s2mMusculoSkeletalModel &m,
            const std::vector<biorbd::utils::Attitude> &IMUobs,
            biorbd::utils::GenCoord *Q,
            biorbd::utils::GenCoord *Qdot,
            biorbd::utils::GenCoord *Qddot);
    virtual void reconstructFrame(
            s2mMusculoSkeletalModel &m,
            const Eigen::VectorXd &IMUobs,
            biorbd::utils::GenCoord *Q,
            biorbd::utils::GenCoord *Qdot,
            biorbd::utils::GenCoord *Qddot);
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

#endif // S2M_KALMAN_RECONS_IMU_H
