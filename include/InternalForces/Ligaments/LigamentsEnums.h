#ifndef BIORBD_LIGAMENTS_ENUMS_H
#define BIORBD_LIGAMENTS_ENUMS_H

namespace BIORBD_NAMESPACE
{
namespace internal_forces
{
namespace ligaments
{

///
/// \brief The available muscle type
///
enum LIGAMENT_TYPE {
    LIGAMENT_CONSTANT,
    LIGAMENT_SPRING_LINEAR,
    LIGAMENT_SPRING_SECOND_ORDER,
    NO_LIGAMENT_TYPE
};

///
/// \brief MUSCLE_TYPE_toStr returns the type name in a string format
/// \param type The type to convert to string
/// \return The name of the type
///
inline const char* LIGAMENT_TYPE_toStr(LIGAMENT_TYPE type)
{
    switch (type) {
    case LIGAMENT_CONSTANT:
        return "Constant";
    case LIGAMENT_SPRING_LINEAR:
        return "Linear";
    case LIGAMENT_SPRING_SECOND_ORDER:
        return "SecondOrder";
    default:
        return "NoType";
    }
}

}
}
}

#endif // BIORBD_MUSCLES_ENUMS_H
