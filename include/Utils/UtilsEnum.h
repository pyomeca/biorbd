#ifndef BIORBD_UTILS_ENUMS_H
#define BIORBD_UTILS_ENUMS_H

namespace biorbd {
namespace utils {

enum NODE_TYPE {
    NODE3D,
    BONE_POINT,
    ROTOTRANS,
    BONE,
    WRAPPING_OBJECT,
    WRAPPING_CYLINDER,
    WRAPPING_SPHERE,
    VIA_POINT,
    NO_NODE_TYPE
};
inline const char* NODE_TYPE_toStr(biorbd::utils::NODE_TYPE type)
{
    switch (type)
    {
    case NODE3D: return "Node3d";
    case BONE_POINT: return "NodeBone";
    case ROTOTRANS: return "RotoTrans";
    case BONE: return "Bone";
    case WRAPPING_OBJECT: return "WrappingObject";
    case WRAPPING_CYLINDER: return "WrappingCylinder";
    case WRAPPING_SPHERE: return "WrappinSphere";
    case VIA_POINT: return "ViaPoint";
    default: return "NoType";
    }
}

}}

#endif // BIORBD_UTILS_ENUMS_H
