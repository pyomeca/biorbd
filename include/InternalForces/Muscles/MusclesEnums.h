#ifndef BIORBD_MUSCLES_ENUMS_H
#define BIORBD_MUSCLES_ENUMS_H

namespace BIORBD_NAMESPACE {
namespace internal_forces {
namespace muscles {

///
/// \brief The available muscle type
///
enum MUSCLE_TYPE {
  IDEALIZED_ACTUATOR,
  HILL,
  HILL_THELEN,
  HILL_THELEN_ACTIVE,
  HILL_THELEN_FATIGABLE,
  HILL_DE_GROOTE_ACTIVE,
  HILL_DE_GROOTE_FATIGABLE,
  HILL_DE_GROOTE,
  NO_MUSCLE_TYPE
};

///
/// \brief MUSCLE_TYPE_toStr returns the type name in a string format
/// \param type The type to convert to string
/// \return The name of the type
///
inline const char* MUSCLE_TYPE_toStr(MUSCLE_TYPE type) {
  switch (type) {
    case IDEALIZED_ACTUATOR:
      return "IdealizedActuator";
    case HILL:
      return "Hill";
    case HILL_THELEN:
      return "Thelen";
    case HILL_THELEN_ACTIVE:
      return "ThelenActive";
    case HILL_THELEN_FATIGABLE:
      return "ThelenFatigable";
    case HILL_DE_GROOTE_ACTIVE:
      return "DeGrooteActive";
    case HILL_DE_GROOTE_FATIGABLE:
      return "DeGrooteFatigable";
    case HILL_DE_GROOTE:
      return "DeGroote";
    default:
      return "NoType";
  }
}

///
/// \brief The available emg state type
///
enum STATE_TYPE { SIMPLE_STATE, DYNAMIC, BUCHANAN, DE_GROOTE, NO_STATE_TYPE };

///
/// \brief STATE_TYPE_toStr returns the type name in a string format
/// \param type The type to convert to string
/// \return The name of the type
///
inline const char* STATE_TYPE_toStr(STATE_TYPE type) {
  switch (type) {
    case SIMPLE_STATE:
      return "Simple";
    case DYNAMIC:
      return "Dynamic";
    case BUCHANAN:
      return "Buchanan";
    default:
      return "NoType";
  }
}

enum STATE_FATIGUE_TYPE {
  SIMPLE_STATE_FATIGUE,
  DYNAMIC_XIA,
  NO_FATIGUE_STATE_TYPE
};
inline const char* STATE_FATIGUE_TYPE_toStr(STATE_FATIGUE_TYPE type) {
  switch (type) {
    case SIMPLE_STATE_FATIGUE:
      return "Simple";
    case DYNAMIC_XIA:
      return "Xia";
    default:
      return "NoType";
  }
}

}  // namespace muscles
}  // namespace internal_forces
}  // namespace BIORBD_NAMESPACE

#endif  // BIORBD_MUSCLES_ENUMS_H
