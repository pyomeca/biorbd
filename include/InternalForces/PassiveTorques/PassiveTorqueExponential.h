#ifndef BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_EXPONENTIAL_H
#define BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_EXPONENTIAL_H

#include "biorbdConfig.h"

#include "InternalForces/PassiveTorques/PassiveTorque.h"

namespace BIORBD_NAMESPACE {
namespace rigidbody {
class GeneralizedCoordinates;
}

namespace internal_forces {
namespace passive_torques {

///
/// \brief Class PassiveTorqueExponential is a joint passive torque type that
/// linearly evolves passive torque is defined by :(B1 * np.exp(k1 * (q-qmid)) +
/// B2 * np.exp(k2 * (q-qmid))) * (1-p_beta*qdot/(s_v*omega_max)) * (q[0] -
/// delta_p) + tau_eq from : https://pubmed.ncbi.nlm.nih.gov/31000347/
class BIORBD_API PassiveTorqueExponential : public PassiveTorque {
 public:
  ///
  /// \brief Construct a linear passive torque
  ///
  PassiveTorqueExponential();

  ///
  /// \brief Construct an exponential passive torque from another passive torque
  /// \param other The other passive torque
  ///
  PassiveTorqueExponential(const PassiveTorqueExponential& other);

  ///
  /// \brief Construct an exponential passive torque
  /// \param k1 The stiffness coefficient of the first exponential
  /// \param k2 The stiffness coefficient of the second exponential
  /// \param b1 The scaling coefficient of the first exponential
  /// \param b2 The scaling coefficient of the second exponential
  /// \param qMid The midpoint angle of the exponentials
  /// \param tauEq The torque offset (tau_eq)
  /// \param pBeta The velocity damping factor (p_beta)
  /// \param wMax The maximal angular velocity (omega_max)
  /// \param sV The velocity scaling factor (s_v)
  /// \param delatP The angle offset (delta_p)
  /// \param dofIdx Index of the DoF associated with passive torque
  ///
  PassiveTorqueExponential(
      const utils::Scalar& k1,
      const utils::Scalar& k2,
      const utils::Scalar& b1,
      const utils::Scalar& b2,
      const utils::Scalar& qMid,
      const utils::Scalar& tauEq,
      const utils::Scalar& pBeta,
      const utils::Scalar& wMax,
      const utils::Scalar& sV,
      const utils::Scalar& delatP,
      size_t dofIdx);

  ///
  /// \brief Construct an exponential passive torque
  /// \param k1 The stiffness coefficient of the first exponential
  /// \param k2 The stiffness coefficient of the second exponential
  /// \param b1 The scaling coefficient of the first exponential
  /// \param b2 The scaling coefficient of the second exponential
  /// \param qMid The midpoint angle of the exponentials
  /// \param tauEq The torque offset (tau_eq)
  /// \param pBeta The velocity damping factor (p_beta)
  /// \param wMax The maximal angular velocity (omega_max)
  /// \param sV The velocity scaling factor (s_v)
  /// \param delatP The angle offset (delta_p)
  /// \param dofIdx Index of the DoF associated with passive torque
  /// \param jointName The name of the parent joint
  ///

  PassiveTorqueExponential(
      const utils::Scalar& k1,
      const utils::Scalar& k2,
      const utils::Scalar& b1,
      const utils::Scalar& b2,
      const utils::Scalar& qMid,
      const utils::Scalar& tauEq,
      const utils::Scalar& pBeta,
      const utils::Scalar& wMax,
      const utils::Scalar& sV,
      const utils::Scalar& delatP,
      size_t dofIdx,
      const utils::String& jointName);

  ///
  /// \brief Destroy the class properly
  ///
  virtual ~PassiveTorqueExponential();

  ///
  /// \brief Deep copy of the linear passive torque
  /// \return A deep copy of the linear passive torque
  ///
  PassiveTorqueExponential DeepCopy() const;

  ///
  /// \brief Deep copy of the linear passive torque from another linear passive
  /// torque
  /// \param other The linear passive torque to copy
  ///
  void DeepCopy(const PassiveTorqueExponential& other);

  ///
  /// \brief Return the passive torque (not available for this type, Q and
  /// QDot must be provided)
  /// \return The passive torque
  ///
  virtual utils::Scalar passiveTorque();

  ///
  /// \brief Return the maximal torque at a given Q
  /// \param Q The generalized coordinates of the passive torque
  /// \param QDot The generalized velocities of the passive torque
  /// \return The maximal torque
  ///
  virtual utils::Scalar passiveTorque(
      const rigidbody::GeneralizedCoordinates& Q,
      const rigidbody::GeneralizedCoordinates& QDot) const;

 protected:
  ///
  /// \brief Set the type of passive torque
  ///
  virtual void setType();

  std::shared_ptr<utils::Scalar> m_k1;     ///< Stiffness of the 1st exponential
  std::shared_ptr<utils::Scalar> m_k2;     ///< Stiffness of the 2nd exponential
  std::shared_ptr<utils::Scalar> m_b1;     ///< Scaling of the 1st exponential
  std::shared_ptr<utils::Scalar> m_b2;     ///< Scaling of the 2nd exponential
  std::shared_ptr<utils::Scalar> m_qMid;   ///< Midpoint angle
  std::shared_ptr<utils::Scalar> m_tauEq;  ///< Torque offset
  std::shared_ptr<utils::Scalar> m_pBeta;  ///< Velocity damping factor
  std::shared_ptr<utils::Scalar> m_wMax;   ///< Maximal angular velocity
  std::shared_ptr<utils::Scalar> m_sV;     ///< Velocity scaling factor
  std::shared_ptr<utils::Scalar> m_deltaP;  ///< Angle offset
};

}  // namespace passive_torques
}  // namespace internal_forces
}  // namespace BIORBD_NAMESPACE

#endif  // BIORBD_PASSIVE_TORQUES_PASSIVE_TORQUE_EXPONENTIAL_H
