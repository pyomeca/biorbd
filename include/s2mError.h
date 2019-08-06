#ifndef S2MMERROR_H
#define S2MMERROR_H


#include "biorbdConfig.h"
class s2mString;
class BIORBD_API s2mError
{
    public:
    static void s2mAssert(bool cond, const s2mString &message);
    static void s2mWarning(bool cond, const s2mString &message);
};
#include "s2mString.h"

#endif // S2MMERROR_H


