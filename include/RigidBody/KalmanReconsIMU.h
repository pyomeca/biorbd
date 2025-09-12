#ifndef BIORBD_RIGIDBODY_KALMAN_RECONS_IMU_H
#define BIORBD_RIGIDBODY_KALMAN_RECONS_IMU_H

#include "biorbdConfig.h"

#include "RigidBody/KalmanRecons.h"

namespace BIORBD_NAMESPACE {
namespace utils {
class RotoTrans;
}

namespace rigidbody {
class IMU;

///
/// \brief Class Kinematic reconstruction algorithm using an Extended Kalman
/// Filter for IMU
///
class BIORBD_API KalmanReconsIMU : public KalmanRecons {
 public:
  // Constructor
  ///
  /// \brief Initialize the Kalman filter and kalman reconstruction for inertial
  /// measurement units (IMU) data
  ///
  KalmanReconsIMU();

  ///
  /// \brief Initialize the Kalman filter and Kalman reconstruction for inertial
  /// measurement units (IMU) data
  /// \param model The joint model
  /// \param params The Kalman filter parameters
  ///
  KalmanReconsIMU(
      Model &model,
      KalmanParam params = KalmanParam(100, 0.005, 1e-10));

  ///
  /// \brief Deep copy of the Kalman reconstruction from inertial measurement
  /// units (IMU) data
  /// \return Copy of the Kalman reconstruction from IMU data
  ///
  KalmanReconsIMU DeepCopy() const;

  ///
  /// \brief Deep copy of a Kalman reconstruction from inertial measurement
  /// units (IMU) data
  /// \param other The Kalman reconstruction to copy
  ///
  void DeepCopy(const KalmanReconsIMU &other);

  // Reconstruction of a frame

  ///
  /// \brief Proceed to one iteration of the Kalman filter
  /// \param model The joint model
  /// \param IMUobs Observed inertial measurement unit (IMU) data
  /// \param Q The generalized coordinates
  /// \param Qdot The generalized velocities
  /// \param Qddot The generalized accelerations
  ///
  virtual void reconstructFrame(
      Model &model,
      const std::vector<IMU> &IMUobs,
      GeneralizedCoordinates *Q,
      GeneralizedVelocity *Qdot,
      GeneralizedAcceleration *Qddot);

  ///
  /// \brief Reconstruct the kinematics
  /// \param model The joint model
  /// \param IMUobs Observed inertial measurement unit (IMU) data in one large
  /// column-major vector
  /// \param Q The generalized coordinates
  /// \param Qdot The generalized velocities
  /// \param Qddot The generalized accelerations
  ///
  virtual void reconstructFrame(
      Model &model,
      const utils::Vector &IMUobs,
      GeneralizedCoordinates *Q,
      GeneralizedVelocity *Qdot,
      GeneralizedAcceleration *Qddot);

  ///
  /// \brief This function cannot be used to reconstruct frames
  ///
  virtual void reconstructFrame();

  ///
  /// \brief Return if the first iteration was done
  /// \return If the first iteration was done
  ///
  bool first();

 protected:
  ///
  /// \brief Manage the occlusion during the iteration
  /// \param InvTp The inverse of the Tp matrix
  /// \param measure The vector actual measurement to track
  /// \param occlusion The vector where occlusions occurs
  ///
  virtual void manageOcclusionDuringIteration(
      utils::Matrix &InvTp,
      utils::Vector &measure,
      const std::vector<size_t> &occlusion);

  std::shared_ptr<bool> m_firstIteration;  ///< If first iteration was done
};

}  // namespace rigidbody
}  // namespace BIORBD_NAMESPACE

#endif  // BIORBD_RIGIDBODY_KALMAN_RECONS_IMU_H
