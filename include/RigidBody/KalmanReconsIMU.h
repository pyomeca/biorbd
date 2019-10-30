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

///
/// \brief Class KalmanReconsIMU that holds the KalmanRecons class 
///
class BIORBD_API KalmanReconsIMU : public biorbd::rigidbody::KalmanRecons
{
public:

    // Constructor
    /// 
    /// \brief Initialize the Kalman filter and kalman reconstruction for inertial measurement units (IMU) data
    ///
    KalmanReconsIMU();

    /// 
    /// \brief Initialize the Kalman filter and Kalman reconstruction for inertial measurement units (IMU) data
    /// \param model The model
    /// \param params The Kalman filter parameters
    ///
    KalmanReconsIMU(
            biorbd::Model& model,
            biorbd::rigidbody::KalmanRecons::KalmanParam = biorbd::rigidbody::KalmanRecons::KalmanParam(100, 0.005, 1e-10));

    ///
    /// \brief Deep copy of the Kalman reconstruction from inertial measurement units (IMU) data
    /// \return Copy of the Kalman reconstruction from inertial measurement units (IMU) data
    ///
    biorbd::rigidbody::KalmanReconsIMU DeepCopy() const;

    ///
    /// \brief Deep copy of a Kalman reconstruction from inertial measurement units (IMU) data
    /// \param other The Kalman reconstruction to copy
    ///
    void DeepCopy(const biorbd::rigidbody::KalmanReconsIMU& other);

    // Reconstruction of a frame

    ///
    /// \brief Reconstruct the kinematics
    /// \param model The model
    /// \param IMUobs Observed inertial measurement unit (IMU) data
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param Qddot The acceleration variables of the model
    ///
    virtual void reconstructFrame(
            biorbd::Model &model,
            const std::vector<biorbd::rigidbody::IMU> &IMUobs,
            biorbd::rigidbody::GeneralizedCoordinates *Q,
            biorbd::rigidbody::GeneralizedCoordinates *Qdot,
            biorbd::rigidbody::GeneralizedCoordinates *Qddot);

    ///
    /// \brief Reconstruct the kinematics 
    /// \param model The model
    /// \param IMUobs Observed inertial measurement unit (IMU) data in one large vector
    /// \param Q The position variables of the model
    /// \param Qdot The velocity variables of the model
    /// \param Qddot The acceleration variables of the model
    ///
    virtual void reconstructFrame(
            biorbd::Model &model,
            const biorbd::utils::Vector &IMUobs,
            biorbd::rigidbody::GeneralizedCoordinates *Q,
            biorbd::rigidbody::GeneralizedCoordinates *Qdot,
            biorbd::rigidbody::GeneralizedCoordinates *Qddot);

    ///
    /// \brief Error message if no inputs in this function
    ///
    virtual void reconstructFrame();

    ///
    /// \brief To know if the initialization of the Kalman filter was done
    /// \return True or False
    ///
    bool first();

protected:
    ///
    /// \brief To initialize the Kalman filter
    ///
    virtual void initialize();

    ///
    /// \brief To manage occlusion during iteration
    /// \invTp The inverse of the Tp matrix
    /// \measure  The measurements
    /// \occlusion The occlusion TODO
    ///
    virtual void manageOcclusionDuringIteration(
            biorbd::utils::Matrix &InvTp,
            biorbd::utils::Vector &measure,
            const std::vector<unsigned int> &occlusion);

    std::shared_ptr<biorbd::utils::Matrix> m_PpInitial; ///< Initial Pp matrix
    std::shared_ptr<bool> m_firstIteration; ///< If first iteration was done (True or False)
};

}}

#endif // BIORBD_RIGIDBODY_KALMAN_RECONS_IMU_H
