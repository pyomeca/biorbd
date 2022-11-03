#ifndef BIORBD_PASSIVE_TORQUES_ENUMS_H
#define BIORBD_PASSIVE_TORQUES_ENUMS_H

namespace BIORBD_NAMESPACE
{
namespace internal_forces
{
namespace passive_torques
{

///
/// \brief Types of passive torques available
///
enum TYPE {
    CONSTANT,
    LINEAR,
//    EXPONENTIAL,
    NO_TYPE
};

///
/// \brief TYPE_toStr returns the type name in a string format
/// \param type The type to convert to string
/// \return The name of the type
///
inline const char* TYPE_toStr(TYPE type)
{
    switch (type) {
    case CONSTANT:
        return "Constant";
    case LINEAR:
        return "Linear";
//    case EXPONENTIAL:
//        return "Exponential";
    default:
        return "NoType";
    }
}

///
/// \brief Types of dof available in passive torque class
///

enum DOF_TYPE {
    ELBOW_FLEXION,
    NO_DOF_TYPE
};

///
/// \brief DOF_TYPE_toStr returns the type name in a string format
/// \param dof_type The dof_type to convert to string
/// \return The name of the dof_type
///
inline const char* DOF_TYPE_toStr(DOF_TYPE dof_type)
{
    switch (dof_type) {
    case ELBOW_FLEXION:
        return "ElbowFlexion";
    default:
        return "NoDofType";
    }
}


}
}
}

#endif // BIORBD_PASSIVE_TORQUES_ENUMS_H
