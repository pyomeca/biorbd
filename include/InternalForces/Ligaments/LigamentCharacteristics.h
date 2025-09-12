#ifndef BIORBD_LIGAMENTS_LIGAMENT_CHARACTERISTICS_H
#define BIORBD_LIGAMENTS_LIGAMENT_CHARACTERISTICS_H

#include "biorbdConfig.h"

#include <cstddef>
#include <memory>

#include "Utils/Scalar.h"

namespace BIORBD_NAMESPACE {
namespace internal_forces {
namespace ligaments {
class State;
class FatigueParameters;
///
/// \brief Class Holds that ligament characteristics
///
class BIORBD_API LigamentCharacteristics {
 public:
  ///
  /// \brief Construct characteristics
  ///
  LigamentCharacteristics();

  ///
  /// \brief Construct characteristics from other characteristics
  /// \param other The other characteristics
  ///
  LigamentCharacteristics(const LigamentCharacteristics& other);

  ///
  /// \brief Construct characteristics
  /// \param rate The rate of the spring like ligament
  /// \param ligamentSlackLength The ligament slack length
  /// \param useDamping Use damping (default: false)
  ///
  LigamentCharacteristics(
      const utils::Scalar& ligamentSlackLength,
      const utils::Scalar& cste_damping = 0,
      const utils::Scalar& cste_maxShorteningSpeed = 1);

  ///
  /// \brief Destroy the class properly
  ///
  virtual ~LigamentCharacteristics();

  ///
  /// \brief Deep copy of characteristics
  /// \return A depp copy of characteristics
  ///
  LigamentCharacteristics DeepCopy() const;

  ///
  /// \brief Deep copy of characteristics from another characteristics
  /// \param other The characteristics to copy from
  ///
  void DeepCopy(const LigamentCharacteristics& other);

  ///
  /// \brief Set the tendon slack length
  /// \param val Value of the tendon slack length
  ///
  void setLigamentSlackLength(const utils::Scalar& val);
  ///
  /// \brief Return the tendon slack length
  /// \return The tendon slack length
  ///
  const utils::Scalar& ligamentSlackLength() const;

  ///
  /// \brief Choose if use damping for ligament force computation
  /// \param val 0 to not use damping
  ///
  void setDampingParam(const utils::Scalar& val);

  ///
  /// \brief Return 1 if use damping for ligament computation
  /// \return 0 if not use damping 1 overall
  ///
  const utils::Scalar& dampingParam() const;

  ///
  /// \brief Choose if use damping for ligament force computation
  /// \param val 0 to not use damping
  ///
  void setMaxShorteningSpeed(const utils::Scalar& val);

  ///
  /// \brief Return 1 if use damping for ligament computation
  /// \return 0 if not use damping 1 overall
  ///
  const utils::Scalar& maxShorteningSpeed() const;

 protected:
  std::shared_ptr<utils::Scalar>
      m_ligamentSlackLength;  ///< Ligament slack length
  std::shared_ptr<utils::Scalar>
      m_cste_damping;  ///< Ligament damping parameters (0 no damping)
  std::shared_ptr<utils::Scalar>
      m_cste_maxShorteningSpeed;  ///< Ligament shorteningSpeed
};

}  // namespace ligaments
}  // namespace internal_forces
}  // namespace BIORBD_NAMESPACE

#endif  // BIORBD_LIGAMENTS_LIGAMENT_CHARACTERISTICS_H
