#ifndef BIORBD_MUSCLES_ENUMS_H
#define BIORBD_MUSCLES_ENUMS_H

namespace biorbd {
namespace muscles {

enum MUSCLE_TYPE {
    IDEALIZED_ACTUATOR,
    HILL,
    HILL_THELEN,
    HILL_THELEN_FATIGABLE,
    NO_MUSCLE_TYPE
};
inline const char* MUSCLE_TYPE_toStr(biorbd::muscles::MUSCLE_TYPE type)
{
    switch (type)
    {
    case IDEALIZED_ACTUATOR: return "IdealizedActuator";
    case HILL: return "Hill";
    case HILL_THELEN: return "Thelen";
    case HILL_THELEN_FATIGABLE: return "ThelenFatigable";
    default: return "NoType";
    }
}

enum STATE_TYPE {
    SIMPLE_STATE,
    DYNAMIC,
    BUCHANAN,
    NO_STATE_TYPE
};
inline const char* STATE_TYPE_toStr(biorbd::muscles::STATE_TYPE type)
{
    switch (type)
    {
    case SIMPLE_STATE: return "Simple";
    case DYNAMIC: return "Dynamic";
    case BUCHANAN: return "Buchanan";
    default: return "NoType";
    }
}

enum STATE_FATIGUE_TYPE {
    SIMPLE_STATE_FATIGUE,
    DYNAMIC_XIA,
    NO_FATIGUE_STATE_TYPE
};
inline const char* STATE_FATIGUE_TYPE_toStr(biorbd::muscles::STATE_FATIGUE_TYPE type)
{
    switch (type)
    {
    case SIMPLE_STATE_FATIGUE: return "Simple";
    case DYNAMIC_XIA: return "Xia";
    default: return "NoType";
    }
}

}}

#endif // BIORBD_MUSCLES_ENUMS_H
