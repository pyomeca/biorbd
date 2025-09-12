#ifndef BIORBD_MUSCLES_STATE_DYNAMICS_DE_GROOTE_H
#define BIORBD_MUSCLES_STATE_DYNAMICS_DE_GROOTE_H

#include "biorbdConfig.h"

#include "InternalForces/Muscles/StateDynamics.h"

namespace BIORBD_NAMESPACE {
namespace internal_forces {
namespace muscles {
///
/// \brief EMG with the capability to compute the time derivative
///
class BIORBD_API StateDynamicsDeGroote : public StateDynamics {
 public:
  ///
  /// \brief Construct the state dynamics
  /// \param excitation The muscle excitation
  /// \param activation The muscle activation
  ///
  StateDynamicsDeGroote(
      const utils::Scalar& excitation = 0,
      const utils::Scalar& activation = 0);

  ///
  /// \brief Construct a state dynamics from another state dynamics
  /// \param other The other state dynamics
  ///
  StateDynamicsDeGroote(const StateDynamicsDeGroote& other);

  ///
  /// \brief Deep copy of state dynamics
  /// \return A deep copy of state dynamics
  ///
  StateDynamicsDeGroote DeepCopy() const;

  ///
  /// \brief Deep copy of state dynamics into another state dynamics
  /// \param other The state dynamics to copy
  ///
  void DeepCopy(const StateDynamicsDeGroote& other);

  ///
  /// \brief Compute and return the activation time derivative
  /// \param characteristics The muscle characteristics
  /// \param alreadyNormalized If already normalized
  /// \return The activation time derivative
  ///
  virtual const utils::Scalar& timeDerivativeActivation(
      const Characteristics& characteristics,
      bool alreadyNormalized = false);

 protected:
  virtual void setType();
};

}  // namespace muscles
}  // namespace internal_forces
}  // namespace BIORBD_NAMESPACE

#endif  // BIORBD_MUSCLES_STATE_DYNAMICS_DE_GROOTE_H
