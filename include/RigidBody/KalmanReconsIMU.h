#ifndef BIORBD_RIGIDBODY_KALMAN_RECONS_IMU_H
#define BIORBD_RIGIDBODY_KALMAN_RECONS_IMU_H

#include "biorbdConfig.h"
#include "RigidBody/KalmanRecons.h"

namespace biorbd {
namespace utils {
class RotoTrans;
}

namespace rigidbody {
class IMU;

class BIORBD_API KalmanReconsIMU : public biorbd::rigidbody::KalmanRecons
{
public:

    // Constructeur
    KalmanReconsIMU();
    KalmanReconsIMU(
            biorbd::Model&,
            biorbd::rigidbody::KalmanRecons::KalmanParam = biorbd::rigidbody::KalmanRecons::KalmanParam(100, 0.005, 1e-10));
    biorbd::rigidbody::KalmanReconsIMU DeepCopy() const;
    void DeepCopy(const biorbd::rigidbody::KalmanReconsIMU& other);

    // Reconstruction d'un frame
    virtual void reconstructFrame(
            biorbd::Model &model,
            const std::vector<biorbd::rigidbody::IMU> &IMUobs,
            biorbd::rigidbody::GeneralizedCoordinates *Q,
            biorbd::rigidbody::GeneralizedCoordinates *Qdot,
            biorbd::rigidbody::GeneralizedCoordinates *Qddot);
    virtual void reconstructFrame(
            biorbd::Model &model,
            const biorbd::utils::Vector &IMUobs,
            biorbd::rigidbody::GeneralizedCoordinates *Q,
            biorbd::rigidbody::GeneralizedCoordinates *Qdot,
            biorbd::rigidbody::GeneralizedCoordinates *Qddot);
    virtual void reconstructFrame();

    bool first();

protected:
    virtual void initialize();
    virtual void manageOcclusionDuringIteration(
            biorbd::utils::Matrix&,
            biorbd::utils::Vector &measure,
            const std::vector<unsigned int> &occlusion);

    std::shared_ptr<biorbd::utils::Matrix> m_PpInitial; // Se souvenir de Pp inital
    std::shared_ptr<bool> m_firstIteration;
};

}}

#endif // BIORBD_RIGIDBODY_KALMAN_RECONS_IMU_H
