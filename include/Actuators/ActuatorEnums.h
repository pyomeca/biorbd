#ifndef BIORBD_ACTUATOR_ENUMS_H
#define BIORBD_ACTUATOR_ENUMS_H

namespace biorbd {
namespace actuator {

enum TYPE {
    CONSTANT,
    LINEAR,
    GAUSS3P,
    GAUSS6P,
    NO_TYPE
};
inline const char* TYPE_toStr(biorbd::actuator::TYPE type)
{
    switch (type)
    {
    case CONSTANT: return "Constant";
    case LINEAR: return "Linear";
    case GAUSS3P: return "Gauss3p";
    case GAUSS6P: return "Gauss6p";
    default: return "NoType";
    }
}


}
}

#endif // BIORBD_ACTUATOR_ENUMS_H
