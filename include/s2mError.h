#ifndef S2MMERROR_H
#define S2MMERROR_H


#include "s2mString.h"
class s2mString;
class s2mError
{
    public:
    static void s2mAssert(bool cond, const s2mString &message);
    static void s2mWarning(bool cond, const s2mString &message);
};

#endif // S2MMERROR_H


