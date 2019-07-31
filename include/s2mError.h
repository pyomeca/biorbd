#ifndef S2M_ERROR_H
#define S2M_ERROR_H


#include "biorbdConfig.h"
#include "s2mString.h"
class s2mString;
class BIORBD_API s2mError
{
    public:
    static void s2mAssert(bool cond, const s2mString &message);
    static void s2mWarning(bool cond, const s2mString &message);
};

#endif // S2M_ERROR_H


