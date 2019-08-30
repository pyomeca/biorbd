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
    NO_TYPE
};

}}

#endif // BIORBD_UTILS_ENUMS_H
