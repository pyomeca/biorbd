#ifndef BIORBD_UTILS_ENUMS_H
#define BIORBD_UTILS_ENUMS_H

namespace BIORBD_NAMESPACE
{
namespace utils
{

///
/// \brief The available node type
///
enum NODE_TYPE {
    VECTOR2D,
    VECTOR3D,
    BONE_POINT,
    ROTOTRANS,
    SEGMENT,
    WRAPPING_OBJECT,
    WRAPPING_HALF_CYLINDER,
    WRAPPING_SPHERE,
    VIA_POINT,
    SOFT_CONTACT,
    SOFT_CONTACT_SPHERE,
    NO_NODE_TYPE
};

///
/// \brief NODE_TYPE_toStr returns the type name in a string format
/// \param type The type to convert to string
/// \return The name of the type
///
inline const char* NODE_TYPE_toStr(
    NODE_TYPE type)
{
    switch (type) {
    case VECTOR3D:
        return "Vector3d";
    case BONE_POINT:
        return "NodeBone";
    case ROTOTRANS:
        return "RotoTrans";
    case SEGMENT:
        return "Segment";
    case WRAPPING_OBJECT:
        return "WrappingObject";
    case WRAPPING_HALF_CYLINDER:
        return "WrappingHalfCylinder";
    case WRAPPING_SPHERE:
        return "WrappinSphere";
    case VIA_POINT:
        return "ViaPoint";
    case SOFT_CONTACT:
        return "SoftContact";
    case SOFT_CONTACT_SPHERE:
        return "SoftContactSphere";
    default:
        return "NoType";
    }
}

}
}

#endif // BIORBD_UTILS_ENUMS_H
