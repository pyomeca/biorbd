#ifndef BIORBD_RIGIDBODY_IMUS_H
#define BIORBD_RIGIDBODY_IMUS_H

#include "biorbdConfig.h"

#include <memory>
#include <vector>

namespace BIORBD_NAMESPACE {
namespace utils {
class String;
class Matrix;
class RotoTransNode;
}  // namespace utils

namespace rigidbody {
class GeneralizedCoordinates;
class IMU;
///
/// \brief Hold a set of IMUs
///
class BIORBD_API IMUs {
 public:
  ///
  /// \brief Construct inertial measurement units set
  ///
  IMUs();

  ///
  /// \brief Construct inertial measurement units set from another set
  /// \param other The other IMU set
  ///
  IMUs(const IMUs& other);

  ///
  /// \brief Destroy the class properly
  ///
  virtual ~IMUs();

  ///
  /// \brief Deep copy of the inertial measurement units data
  /// \return A copy of the inertial measurement units data
  ///
  IMUs DeepCopy() const;

  ///
  /// \brief Deep copy the inertial measurement units data
  /// \param other The IMU data to copy
  ///
  void DeepCopy(const IMUs& other);

  // Set and get

  ///
  /// \brief Add a new inertial measurement unit to the set
  /// \param technical True if the IMU is technical
  /// \param anatomical True if the IMU is anatomical
  ///
  void addIMU(bool technical = true, bool anatomical = true);

  ///
  /// \brief Add a new inertial measurement unit to the set
  /// \param RotoTrans The RotaTrans of the IMU
  /// \param technical True if the IMU is technical
  /// \param anatomical True if the IMU is anatomical
  ///
  void addIMU(
      const utils::RotoTransNode& RotoTrans,
      bool technical = true,
      bool anatomical = true);

  ///
  /// \brief Return the number of inertial measurement units (IMU) in the set
  /// \return The number of IMU
  ///
  size_t nbIMUs() const;

  ///
  /// \brief Return the names of the inertial measurement units (IMU)
  /// \return The names of the IMU
  ///
  std::vector<utils::String> IMUsNames();

  ///
  /// \brief Return the names of the technical inertial measurement units
  /// (IMU)
  /// \return The names of the technical IMU
  ///
  std::vector<utils::String> technicalIMUsNames();

  ///
  /// \brief Return the names of the anatomical inertial measurement units
  /// (IMU)
  /// \return The names of the anatomical IMU
  ///
  std::vector<utils::String> anatomicalIMUsNames();

  ///
  /// \brief Return all the IMU in the local reference of the segment
  /// \return All the inertial measurement units in local reference frame
  ///
  const std::vector<rigidbody::IMU>& IMU() const;

  ///
  /// \brief Return all the inertial measurement units (IMU) of a segment
  /// \param segmentName The name of the segment to return the IMU
  /// \return All the IMU of attached to the segment
  ///
  std::vector<rigidbody::IMU> IMU(const utils::String& segmentName);

  ///
  /// \brief Return the inertial measurement unit (IMU) of a specified index
  /// \param idx The index of the IMU in the set
  /// \return IMU of idx i
  ///
  const rigidbody::IMU& IMU(size_t idx);

  ///
  /// \brief Compute and return all the inertial measurement units (IMU) at
  /// the position given by Q
  /// \param Q The generalized coordinates
  /// \param updateKin If the model should be updated
  /// \return All the IMU at the position given by Q
  ///
  std::vector<rigidbody::IMU> IMU(
      const GeneralizedCoordinates& Q,
      bool updateKin = true);

  ///
  /// \brief Compute and return one inertial meausrement unit (IMU) at the
  /// position given by Q
  /// \param Q The generalized coordinates
  /// \param idx The index of the IMU in the set
  /// \param updateKin If the model should be updated
  /// \return The IMU of index idx at the position given by Q
  ///
  rigidbody::IMU
  IMU(const GeneralizedCoordinates& Q, size_t idx, bool updateKin = true);

  ///
  /// \brief Return all the inertial measurement units (IMU) on a specified
  /// segment
  /// \param Q The generalized coordinates
  /// \param idx The index of the segment
  /// \param updateKin If the model should be updated
  /// \return All the IMU on the segment of index idx
  ///
  std::vector<rigidbody::IMU> segmentIMU(
      const GeneralizedCoordinates& Q,
      size_t idx,
      bool updateKin = true);

  ///
  /// \brief Return the number of technical inertial measurement units (IMU)
  /// \return The number of technical IMU
  ///
  size_t nbTechIMUs();

  ///
  /// \brief Return the number of anatomical inertial measurement units (IMU)
  /// \return The number of anatomical IMU
  ///
  size_t nbAnatIMUs();

  ///
  /// \brief Return all the technical inertial measurement units (IMU)
  /// \param Q The generalized coordinates
  /// \param updateKin If the model should be updated
  /// \return all the technical IMU
  ///
  std::vector<rigidbody::IMU> technicalIMU(
      const GeneralizedCoordinates& Q,
      bool updateKin = true);

  ///
  /// \brief Return all the technical inertial measurement units (IMU) in
  /// their respective segment local reference frame
  /// \return All the technical IMU
  ///
  std::vector<rigidbody::IMU> technicalIMU();

  ///
  /// \brief Return all the anatomical inertial measurement units (IMU)
  /// \param Q The generalized coordinates
  /// \param updateKin If the model should be updated
  /// \return all the anatomical IMU
  ///
  std::vector<rigidbody::IMU> anatomicalIMU(
      const GeneralizedCoordinates& Q,
      bool updateKin = true);

  ///
  /// \brief Return all the anatomical inertial measurement units (IMU) in
  /// their respective segment local reference frame
  /// \return All the anatomical IMU
  ///
  std::vector<rigidbody::IMU> anatomicalIMU();

  ///
  /// \brief Return the jacobian of the inertial measurement units (IMU)
  /// \param Q The generalized coordinates
  /// \param updateKin If the model should be updated
  /// \return The jacobien of the IMU
  ///
  std::vector<utils::Matrix> IMUJacobian(
      const GeneralizedCoordinates& Q,
      bool updateKin = true);

  ///
  /// \brief Return the jacobian of the technical inertial measurement units
  /// (IMU)
  /// \param Q The generalized coordinates
  /// \param updateKin If the model should be updated
  /// \return The jacobian of the technical IMU
  ///
  std::vector<utils::Matrix> TechnicalIMUJacobian(
      const GeneralizedCoordinates& Q,
      bool updateKin = true);

 protected:
  ///
  /// \brief Compute and return the jacobian of all the inertial measurement
  /// units (IMU)
  /// \param Q The generalized coordinates
  /// \param updateKin If the model should be updated
  /// \param lookForTechnical If true, only computes for the technical IMU
  ///
  std::vector<utils::Matrix> IMUJacobian(
      const GeneralizedCoordinates& Q,
      bool updateKin,
      bool lookForTechnical);

  std::shared_ptr<std::vector<rigidbody::IMU>>
      m_IMUs;  ///< All the inertial Measurement Units
};

}  // namespace rigidbody
}  // namespace BIORBD_NAMESPACE

#endif  // BIORBD_RIGIDBODY_IMUS_H
