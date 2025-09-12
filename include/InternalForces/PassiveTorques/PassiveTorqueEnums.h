#ifndef BIORBD_PASSIVE_TORQUES_ENUMS_H
#define BIORBD_PASSIVE_TORQUES_ENUMS_H

namespace BIORBD_NAMESPACE {
namespace internal_forces {
namespace passive_torques {

///
/// \brief Types of passive torques available
///
enum TORQUE_TYPE {
  TORQUE_CONSTANT,
  TORQUE_LINEAR,
  TORQUE_EXPONENTIAL,
  NO_TORQUE_TYPE
};

///
/// \brief TYPE_toStr returns the type name in a string format
/// \param type The type to convert to string
/// \return The name of the type
///
inline const char* TORQUE_TYPE_toStr(TORQUE_TYPE type) {
  switch (type) {
    case TORQUE_CONSTANT:
      return "Constant";
    case TORQUE_LINEAR:
      return "Linear";
    case TORQUE_EXPONENTIAL:
      return "Exponential";
    default:
      return "NoType";
  }
}

}  // namespace passive_torques
}  // namespace internal_forces
}  // namespace BIORBD_NAMESPACE

#endif  // BIORBD_PASSIVE_TORQUES_ENUMS_H
